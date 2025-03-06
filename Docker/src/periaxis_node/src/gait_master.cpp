#define PARENT_NODE_NAME "gait_controller"

#include "periaxis_node/gait_master.hpp"

// ## PRIVATE GaitMaster CLASS VARIABLES ##
// STATIC RANGE LIMITS FOR GAIT PARAMETERS
const int GaitMaster::_cyclePeriod_millis_MIN = 1000;
const int GaitMaster::_cyclePeriod_millis_MAX = 60000;
const int GaitMaster::_traceRadius_MIN = 10;
const int GaitMaster::_traceRadius_MAX = 150;
const double GaitMaster::_step_offsetFactor_MIN = 1;
const double GaitMaster::_step_offsetFactor_MAX = 50;
const double GaitMaster::_push_ratio_MIN = 0.1;
const double GaitMaster::_push_ratio_MAX = 0.9;
// master trace translation limits
const double GaitMaster::_x_transRangeLimit_abs = 80;
const double GaitMaster::_y_transRangeLimit_abs = 80;
const double GaitMaster::_z_transRangeLimit_MIN = 50;
const double GaitMaster::_z_transRangeLimit_MAX = 200;


// LegController.init: LegController represents an individual leg of the robot. Each instance maintains its own state but shares a common ROS2 Node and publisher from parent GaitMaster instance
LegController::LegController(
		std::shared_ptr<GaitMaster> gaitMaster_node,
		const rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr legGoalCoord_pub,
		const CLK::time_point &clock_master,
		int legNum,
		double legZAngle_rad,
		double legPhaseOffset_ratio,
		int legPhaseOffset_count
	) : _gaitMaster_node(gaitMaster_node),
		_legGoalCoord_pub(legGoalCoord_pub),
		_CLK_stepStart(clock_master),
		_legNum(legNum),
		_legZAngle_rad(legZAngle_rad) 
	{
		//Init class variables
		//Transformation matrix for trace frame offset for leg
		Eigen::Matrix3d _legOffset_rot = Eigen::Matrix3d::Identity(); //rotation matrix to align function with leg angle
		//homogeneous tranformation matrix represents a rotation FOLLOWED by a translation
		Eigen::Matrix4d _A_traceFrame = Eigen::Matrix4d::Identity(); //final transformation matrix to be constructed; set to Identity matrix to make bottom row 0,0,0,1  
		//Track the current position in the circles cycle
		double _PI_current_trace = 0;
		
		//Initialise rotation matrix to place the trace over the corresponding leg
		_legOffset_rot <<   cos(_legZAngle_rad),  -sin(_legZAngle_rad), 0,
							sin(_legZAngle_rad),  cos(_legZAngle_rad),  0,
							0,                    0,                    1;
		
		double numCyclesAdjust = 0; //holds the number of cycle periods the modf normalisation has adjusted for
		//As the calculated phaseOffset_total can be more than a single cycle period, we take only the decimal part of the calculation; eg offset of -120% is equivelant to -20% in current cycle
		double phaseOffset_normalised = std::modf((legPhaseOffset_ratio * legPhaseOffset_count), &numCyclesAdjust);
		
		//Use parameters to initialise transformation matrix and timing offsets for this specific leg;
		// note on the '1-': as we calculate the _CLK_stepStart as a subtraction from the master clock, we need to invert the phaseOffset_normalised,
		// ie -40% offset equates to the leg being 60% of the way through the cycle
		this->initialiseLegParameters(1 - phaseOffset_normalised);
			
		// check if _gaitMaster_node is available using std::weak_ptr (a non-owning reference to a std::shared_ptr); when the std::shared_ptr is still valid, .lock() returns a valid std::shared_ptr; otherwise, it returns nullptr and skips 'if' when ref doesnt exist
		auto parent_node = _gaitMaster_node.lock();
		if (parent_node) {
			// ROS 2 log current leg config
			RCLCPP_INFO(parent_node->get_logger(), "@LegController class constructor: Leg [%d] sucessfully initialised to %s node.", legNum, parent_node->get_name());
		} else {
			RCLCPP_ERROR(rclcpp::get_logger(PARENT_NODE_NAME), "@LegController class constructor: Failed to lock GaitMaster node pointer from leg [%d], pointer to parent Node is invalid!", _legNum);
		}
	}

// LegController.PUBLIC: Leg gait controller executing function to publish leg goal coord
void LegController::publishLegGoal(const CLK::time_point &CLK_master, const bool &me_reInitialise) {
	//create milis duration object called 'elapsed' containing the current duration since the start of the legs cycle
	duration_milis elapsed = CLK_master - _CLK_stepStart;

	//current step cycle progress as a percentage
	double cycle_currentProgress = elapsed.count() / _cyclePeriod_millis;

	//If new parameters have changed, re-initialise object variables with new values
	if(me_reInitialise) {
		_CLK_stepStart = CLK_master; //reset the legs clock to master in preperation for re-initialisation calculation
		this->initialiseLegParameters(cycle_currentProgress); //reinitialise _CLK_stepStart timing with new parameters
	}
	
	// check if weak_ptr to _gaitMaster_node is valid, .lock() returns nullptr and skips 'if' when ref to shared_ptr of parent doesnt exist
	auto parent_node = _gaitMaster_node.lock();
	if (parent_node) {
		//create message instance to publish topics to periaxis_node/legGoalCoord
		periaxis_interfaces::msg::LegGoalCoord legGoalCoord_msg;
	
		/*5 legs; 3 are in contact with the ball at all times, each leg offset by 2/5, or 60% pushing, 40% stepping; to achieve transfer between tips, leg must reach contact point in time to complete step, ie. Previous leg is out of phase by 40% of cycle, in other words 360deg*40% = 144 deg offset between legs.*/
		if(cycle_currentProgress <= _push_ratio) { // ## PUSHING PART OF CYCLE ##
			//Current leg tip trace location coordinates; point must travel from traceSTART to traceEND within (_push_ratio * cyclePeriod_millis),
			// we use (cycle_currentProgress/_push_ratio) to proportion that ratio back to a 0 - 100% range for controlling the
			// traces current position based on cycle_currentProgress
			_PI_current_trace = _traceSTART + ((cycle_currentProgress/_push_ratio) * (_traceEND - _traceSTART));

			//Calculate trace position in base frame of robot
			_traceCoord_base << _traceRadius*cos(_PI_current_trace), 
								_traceRadius*sin(_PI_current_trace),
								0,
								1;
			
			RCLCPP_DEBUG_STREAM(parent_node->get_logger(), 
				"PUSHING: " << _PI_current_trace << " cycle_currentProgress: " << cycle_currentProgress);
			
		} else if(cycle_currentProgress > _push_ratio && cycle_currentProgress < 1) { // ## STEPPING PART OF CYCLE ##
			//Calculates the current percentage progress through the step cycle
			double STEP_progress =  1 - (1-cycle_currentProgress) / (1-_push_ratio);

			//So that the stepping cycle has a soft begining and end, the current progress is adjusted using cosine from pi to 0, this converts the 0-100% STEP_progress into a 100-0% progress which begins and ends slower than the middle of the step
			double PI_stepSoftener = cos(STEP_progress * M_PI)/2 + 0.5;

			//As PI_stepSoftener goes from 100-0%, our trace progresses from traceEND to traceSTART over the (1-_push_ratio) time period to reset the push cycle
			_PI_current_trace = _traceSTART + PI_stepSoftener * (_traceEND - _traceSTART);

			//Calculate trace position at base frame; the step offset part of the equation modifies the y and z coordinate to step away from the circle trace, increasing as the STEP trace position gets to its midway point, and then decreasing again to meet the circle trace to start the next PUSH cycle
			_traceCoord_base << _traceRadius*cos(_PI_current_trace), 
								_traceRadius*sin(_PI_current_trace) + _step_offsetFactor*(sin(_PI_current_trace) - sin(_traceSTART)),
								-_step_offsetFactor*(sin(_PI_current_trace) - sin(_traceSTART)),
								1;

			RCLCPP_DEBUG_STREAM(parent_node->get_logger(), 
				"STEPPING: " << _PI_current_trace << " cycle_currentProgress: " << cycle_currentProgress);

		} else { // ## END OF CYCLE, START NEXT ##
			//reset clock counter
			_CLK_stepStart = CLK_master;
		}

		//Calculate new coords of trace after A frame transform to leg position
		_traceCoord_trans = _A_traceFrame * _traceCoord_base;

		//Set publish message variables with calculated target coordinate
		legGoalCoord_msg.leg_num = _legNum;
		legGoalCoord_msg.coord.x = _traceCoord_trans(0);
		legGoalCoord_msg.coord.y = _traceCoord_trans(1);
		legGoalCoord_msg.coord.z = _traceCoord_trans(2);
		
		//publish LegGoalCoord message
		_legGoalCoord_pub->publish(legGoalCoord_msg);
		
		//send log to ROS2 node
		RCLCPP_INFO_STREAM(parent_node->get_logger(), 
			"Leg " << _legNum 
			<< " current cycle progress " << round(cycle_currentProgress*100) << "% " 
			<< (cycle_currentProgress > _push_ratio ? "STEPPING" : "PUSHING"));
		
	} else {
		RCLCPP_ERROR(rclcpp::get_logger(PARENT_NODE_NAME), "@publishLegGoal: Failed to lock GaitMaster node pointer from leg [%d], pointer to parent Node is invalid!", _legNum);
	}
}

// LegController.PRIVATE: init tracked parameters related to leg timing and the transform to locate the traced function path above this leg
void LegController::initialiseLegParameters(const double& currentCycleProgress) {
	// check if weak_ptr to _gaitMaster_node is valid, .lock() returns nullptr and skips 'if' when ref to shared_ptr of parent doesnt exist
	auto parent_node = _gaitMaster_node.lock();
	if (parent_node) {

		//Update the local class instance versions of gait parameters to reflect new timing and config
		// TBD may be better as a struct datatype
		_cyclePeriod_millis = parent_node->get_cyclePeriod_millis();
		_traceRadius = parent_node->get_traceRadius();
		_step_offsetFactor = parent_node->get_step_offsetFactor();
		_push_ratio = parent_node->get_push_ratio();
		_traceSTART = parent_node->get_traceSTART();
		_traceEND = parent_node->get_traceEND();
		
		//ms_offset_total = _cyclePeriod_millis * currentCycleProgress

		//send log to ROS2 node
		RCLCPP_DEBUG_STREAM(parent_node->get_logger(), 
			"Initialising leg " << _legNum << " progress " << currentCycleProgress 
			<< " new offset applied " << _cyclePeriod_millis * currentCycleProgress << "ms" 
			<< std::chrono::duration_cast<std::chrono::milliseconds>(CLK::now() - _CLK_stepStart).count() / _cyclePeriod_millis << ".");

		//Calculate the new step start based on the current cycle progress from global cycle period value
		_CLK_stepStart -= std::chrono::milliseconds((int)(_cyclePeriod_millis * currentCycleProgress));          

		//Build the complete trace frame transformation matrix
		_A_traceFrame.block<3,3>(0,0) << (parent_node->get_traceOffset_rot() * _legOffset_rot); //combine both the trace frame rotation and the leg rotation (pre multiply _traceOffset_rot as trace frame rotation is about fixed frame)
		_A_traceFrame.block<3,1>(0,3) << parent_node->get_traceOffset_trans();
	
	} else {
		RCLCPP_ERROR(rclcpp::get_logger(PARENT_NODE_NAME), "@initialiseLegParameters: Failed to lock GaitMaster node pointer from leg [%d], pointer to parent Node is invalid!", _legNum);
	}
}


// GaitMaster.init: GaitMaster class inherits and represents the ROS2 Node gait_controller, class configures and manages a number of child instances of the LegController class for each leg of the robot
GaitMaster::GaitMaster() : Node(PARENT_NODE_NAME),
	// Set class const parameters
	_leg_count(5),
	_traceSTART(0.5*M_PI - (1/_leg_count)*M_PI), // 90deg + half of each legs portion of the full 360deg circle
	_traceEND(0.5*M_PI + (1/_leg_count)*M_PI) // 90deg - half of each legs portion of the full 360deg circle
	{ 
	// Initialisation code...
	
	// Master clock reference
	_CLK_master = CLK::now();
	_reInitialiseFlag = false;
	
	// Set initial gait variable parameters
	_publishRate_ms = 500;
	_cyclePeriod_millis = 3500;
	_traceRadius = 5;
	_step_offsetFactor = 10;
	_push_ratio = 0.6;

	// Master trace transforms
	_traceOffset_trans = {0, 0, 15};
	_traceOffset_rot = Eigen::Matrix3d::Identity();

	// Log node state to ROS2
	RCLCPP_INFO(this->get_logger(), "@GaitMaster class constructor: Starting %s node. Node controls position and timing of legs by publishing leg goal coords on periaxis_node/legGoalCoord;", this->get_name());
	
	// Declare ROS 2 parameter references and init class instance variables to default values
	//this->declare_parameter<int>("leg_count", 5);  // name, default
	//_leg_count = this->get_parameter("leg_count").as_int(); // store default to class instance
	
	this->declare_parameter<int>("publishRate_ms", 500);
	this->set_publishRate_ms(this->get_parameter("publishRate_ms").as_int());
	
	this->declare_parameter<int>("cyclePeriod_ms", 3500);
	this->set_cyclePeriod_millis(this->get_parameter("cyclePeriod_ms").as_int());

	this->declare_parameter<int>("traceRadius_mm", 5);
	this->set_traceRadius(this->get_parameter("traceRadius_mm").as_int());
	
	this->declare_parameter<double>("step_offsetFactor", 10);
	this->set_step_offsetFactor(this->get_parameter("step_offsetFactor").as_double());
	
	this->declare_parameter<double>("push_ratio", 0.6);
	this->set_push_ratio(this->get_parameter("push_ratio").as_double());
	
	// BASE FRAME TRACE TRANSLATION
	this->declare_parameter<std::vector<double>>("baseFrame_fnTrace_translationVect", {0.0, 0.0, 15.0});
	auto translation_vect = this->get_parameter("baseFrame_fnTrace_translationVect").as_double_array();
	_traceOffset_trans = Eigen::Vector3d(translation_vect[0], translation_vect[1], translation_vect[2]);

	// BASE FRAME TRACE ROTATION
	this->declare_parameter<std::vector<double>>("baseFrame_fnTrace_rotationQ", {1.0, 0.0, 0.0, 0.0});
	auto rotationQ = this->get_parameter("baseFrame_fnTrace_rotationQ").as_double_array();
	Eigen::Quaterniond q(rotationQ[0], rotationQ[1], rotationQ[2], rotationQ[3]);
	_traceOffset_rot = q.toRotationMatrix();

	// Create shared publisher for all legs
	_legGoalCoord_pub = this->create_publisher<periaxis_interfaces::msg::LegGoalCoord>("periaxis_node/legGoalCoord", 10);
		
	// Create subscriber for dynamic parameter updates
	_gaitParameters_sub = this->create_subscription<periaxis_interfaces::msg::GaitParameters>(
		"periaxis_node/gait_params", 10, // Topic name, ROS2 subscription queue size
		[this](const periaxis_interfaces::msg::GaitParameters::SharedPtr msg) { // const pass-by-ref (resolves to &msg in class function)
			this->parameterCallback(msg);
		}
	);

}

// GaitMaster.PUBLIC: Call this AFTER the node is created so a valid std::make_shared<GaitMaster>() can be passed to legController instances
void GaitMaster::initialiseLegInstances() {
	//log class constructor complete
	RCLCPP_INFO(this->get_logger(), "@initialiseLegInstances: %s initialising %d legs and %dms publish rate.", this->get_name(), _leg_count, _publishRate_ms);
	
	// Get shared_ptr from this class instance; cast to resolve ambiguity with inhereted rclcpp::Node and get a shared_ptr<GaitMaster>
	auto gaitMaster_node_shared = std::static_pointer_cast<GaitMaster>(this->shared_from_this());
	//zOffsetUnit is negative due leg numbers increment CW about Z [+Z up from legs common centre]
	double zOffsetUnit = -(2*M_PI)/_leg_count; //(circle / number of legs)

	// Init child LegController instances; unique_ptr ensures ownership is managed by GaitMaster
	for (int i = 0; i < _leg_count; ++i) {
		// LegController(x, x, x, legNum, legZAngle_rad, legPhaseOffset_ratio, legPhaseOffset_count)
		_legControllers.emplace_back(std::make_unique<LegController>(
			gaitMaster_node_shared,
			_legGoalCoord_pub,
			_CLK_master,
			i + 1,
			i * zOffsetUnit,
			0.4,
			i
		));
	}
	
	// Timer to periodically trigger leg controller updates	
	_publish_timer = this->create_wall_timer(
		std::chrono::milliseconds(_publishRate_ms),
		[this]() {  // lamda function captures "this" node instance to execute all child leg controllers on given publish_rate schedule
			this->updateGaits();
		}
	);
}

// GaitMaster.PRIVATE: Timer driven publishing loop to generate goal coords for each leg
void GaitMaster::updateGaits() {
	//update master clock with current time
	_CLK_master = CLK::now();

	//to ensure all leg instances have their parameters re-initialised together as part of the same update loop after a parameter callback, store current value of _reInitialiseFlag at this instance to avoid callback triggering an update to a subset of the legs part way through a loop
	bool reInitFlagAtLoopStart = _reInitialiseFlag;

	// Calls publishLegGoal for each child LegController instance
	// Generate new leg goal coords for current time slice; ref to current master clock
	for(auto &leg : _legControllers) {
		leg->publishLegGoal(_CLK_master, reInitFlagAtLoopStart);
	}

	// remove leg reinit flag if it was set at loop start, otherwise reinit on next loop
	if(reInitFlagAtLoopStart) {
		_reInitialiseFlag = false;
	}
}

// GaitMaster.PRIVATE: Callback for updating parameters based on messages received on periaxis_node/gait_params
void GaitMaster::parameterCallback(const periaxis_interfaces::msg::GaitParameters::SharedPtr &msg) {
	RCLCPP_INFO(this->get_logger(), "@parameterCallback: Processing new GaitParameters recieved on periaxis_node/gait_params");
	
	//Ensure the recieved msg has the corresponding ARMED bit true, this is to ensure variables are only updated from topics when they are intended
	if(!msg->arm_live_update) {
		RCLCPP_WARN(this->get_logger(), "@parameterCallback: arm_live_update bit is not set, no updates took place.");
		return; //no updates took place
	}

	// Check if _reInitialiseFlag is false so that any previously recieved updates are processed first before a new parameter update is considered
	if(!_reInitialiseFlag) {

		// Set internal master gait parameters
		set_cyclePeriod_millis(msg->cycle_period_millis);
		set_traceRadius(msg->trace_radius);
		set_step_offsetFactor(msg->step_offset_factor);
		set_push_ratio(msg->push_ratio);
		
		// Convert recieved Point message to Eigen::Vector3d
		Eigen::Vector3d vec_translation(msg->trace_offset_trans.x, msg->trace_offset_trans.y, msg->trace_offset_trans.z);
		set_traceOffset_trans(vec_translation); //validate and set internal master gait parameter

		//Convert recieved Quaternion message to Eigen::Quaterniond
		Eigen::Quaterniond q_rotation(msg->trace_offset_rot.w, msg->trace_offset_rot.x, msg->trace_offset_rot.y, msg->trace_offset_rot.z);
		set_traceOffset_rot(q_rotation); //validate and set internal master gait parameter

		//flag reinit required for legs
		_reInitialiseFlag = true;
		
		RCLCPP_INFO(this->get_logger(), "@parameterCallback: New gait parameters have been processed and stored.");
	} else {
		// existing re-init from previous parameter change is waiting to be processed //
		RCLCPP_WARN(this->get_logger(), "@parameterCallback: A previous parameter update is waiting to be processed, no updates took place.");
		return; //no updates took place
	}
}

// ## GaitMaster.Private: SET PRIVATE INTERNAL GAIT PARAMETERS ##
// set internal master gait parameter _publishRate_ms
bool GaitMaster::set_publishRate_ms(const int newValue) {
	bool existingTimer = (_publish_timer != nullptr); //evaluates false _publish_timer == nullptr
	// Only set value if new or first init of _publish_timer
	if(_publishRate_ms == newValue && existingTimer) { return true; }
	// validate within acceptable range
	if(newValue >= 1 && newValue <= 100000) {
		_publishRate_ms = newValue;
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("publishRate_ms", _publishRate_ms));
		
		// Check if the timer is init
		if (existingTimer) {
			_publish_timer->cancel(); //cancel existing
		}
		// Reconfigure pub timer with the new rate
		_publish_timer = this->create_wall_timer(
			std::chrono::milliseconds(_publishRate_ms),
			[this]() { // Re-bind publish timer callback function
				this->updateGaits();
			});
		
		return true;
	}
	RCLCPP_WARN(this->get_logger(), "@set_publishRate_ms: Invalid parameter [%d] is out of range, it was not updated.", newValue);
	return false;
}
	
// set internal master gait parameter _cyclePeriod_millis
bool GaitMaster::set_cyclePeriod_millis(const int newValue) {
	// Only set value if new
	if(_cyclePeriod_millis == newValue) { return true; }
	// validate within the acceptable range
	if(newValue >= _cyclePeriod_millis_MIN && newValue <= _cyclePeriod_millis_MAX) {
		_cyclePeriod_millis = newValue;
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("cyclePeriod_ms", _cyclePeriod_millis));
		return true;
	}
	RCLCPP_WARN(this->get_logger(), "@set_cyclePeriod_millis: Invalid parameter [%d] is out of range, it was not updated.", newValue);
	return false;
}

// set internal master gait parameter _traceRadius
bool GaitMaster::set_traceRadius(const int newValue) {
	// Only set value if new
	if(_traceRadius == newValue) { return true; }
	// validate within the acceptable range
	if(newValue >= _traceRadius_MIN && newValue <= _traceRadius_MAX) {
		_traceRadius = newValue;
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("traceRadius_mm", _traceRadius));
		return true;
	}
	RCLCPP_WARN(this->get_logger(), "@set_traceRadius: Invalid parameter [%d] is out of range, it was not updated.", newValue);
	return false;
}

// set internal master gait parameter _step_offsetFactor
bool GaitMaster::set_step_offsetFactor(const double newValue) {
	// Only set value if new
	if(_step_offsetFactor == newValue) { return true; }
	// validate within the acceptable range
	if(newValue >= _step_offsetFactor_MIN && newValue <= _step_offsetFactor_MAX) {
		_step_offsetFactor = newValue;
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("step_offsetFactor", _step_offsetFactor));
		return true;
	}
	RCLCPP_WARN(this->get_logger(), "@set_step_offsetFactor: Invalid parameter [%f] is out of range, it was not updated.", newValue);
	return false;
}

// set internal master gait parameter _push_ratio
bool GaitMaster::set_push_ratio(const double newValue) {
	// Only set value if new
	if(_push_ratio == newValue) { return true; }
	// validate within the acceptable range
	if(newValue >= _push_ratio_MIN && newValue <= _push_ratio_MAX) {
		_push_ratio = newValue;
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("push_ratio", _push_ratio));
		return true;
	}
	RCLCPP_WARN(this->get_logger(), "@set_push_ratio: Invalid parameter [%f] is out of range, it was not updated.", newValue);
	return false;
}

// set internal master gait parameter _traceOffset_trans
bool GaitMaster::set_traceOffset_trans(const Eigen::Vector3d& new_translation) { // only set new value if within acceptable range
	// Validate that new_translation coordinates are not NaN
	if (std::isnan(new_translation.x()) || std::isnan(new_translation.y()) || std::isnan(new_translation.z())) {
		RCLCPP_WARN(this->get_logger(), "@set_traceOffset_trans: Invalid frame translation recieved, it was not updated.");
		return false;
	}
	
	// Check for out-of-range values
	if (std::abs(new_translation.x()) > _x_transRangeLimit_abs || std::abs(new_translation.y()) > _y_transRangeLimit_abs || 
		new_translation.z() < _z_transRangeLimit_MIN || new_translation.z() > _z_transRangeLimit_MAX) {
		RCLCPP_WARN(this->get_logger(), "@set_traceOffset_trans: Frame translation (%f, %f, %f) is out of range, it was not updated.", new_translation.x(), new_translation.y(), new_translation.z());
		return false;
	}
	
	// Update the local trace frame translation vector
	_traceOffset_trans = new_translation;
	// Update ROS 2 parameter reference
	this->set_parameter(rclcpp::Parameter("baseFrame_fnTrace_translationVect", 
		std::vector<double>{_traceOffset_trans.x(), _traceOffset_trans.y(), _traceOffset_trans.z()}));
	RCLCPP_DEBUG_STREAM(this->get_logger(), "@set_traceOffset_trans: Updated frame translation: " << _traceOffset_trans.transpose());
	return true;
}

// set internal master gait parameter _traceOffset_rot
bool GaitMaster::set_traceOffset_rot(const Eigen::Quaterniond& q) { // only set new value if within acceptable range
	// Check if quaternion is valid, not zero-length or containing NaN or Inf values
	double q_normValue = q.norm();
	if(q_normValue == 0 ||
		std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) ||
		std::isinf(q.w()) || std::isinf(q.x()) || std::isinf(q.y()) || std::isinf(q.z())) {
		RCLCPP_WARN(this->get_logger(), "@set_traceOffset_rot: Received invalid quaternion. Skipping frame rotation.");
		return false;
	}

	// Normalise quaternion to unit vector if not already
	if(q_normValue < 0.9999 || q_normValue > 1.0001) {
		// Normalise and convert to rotation matrix
		Eigen::Quaterniond q_unitNormalised = q.normalized();
		_traceOffset_rot = q_unitNormalised.toRotationMatrix(); 
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("baseFrame_fnTrace_rotationQ", 
			std::vector<double>{q_unitNormalised.w(), q_unitNormalised.x(), q_unitNormalised.y(), q_unitNormalised.z()}));

		RCLCPP_DEBUG_STREAM(this->get_logger(), "@set_traceOffset_rot: Inbound quaternion [" << q.coeffs().transpose() << "] normalised to [" << q_unitNormalised.coeffs().transpose() << "] then processed and converted to rotation matrix:\n" << _traceOffset_rot); // Log new frame rotation details
	} else {
		// Store as rotation matrix
		_traceOffset_rot = q.toRotationMatrix();
		// Update ROS 2 parameter reference
		this->set_parameter(rclcpp::Parameter("baseFrame_fnTrace_rotationQ", 
			std::vector<double>{q.w(), q.x(), q.y(), q.z()}));	
		
		RCLCPP_DEBUG_STREAM(this->get_logger(), "@set_traceOffset_rot: Quaternion [" << q.coeffs().transpose() << "] processed and converted to rotation matrix:\n" << _traceOffset_rot); // Log new frame rotation details
	}
	return true;
}
// ## NEXT SECTION ##
