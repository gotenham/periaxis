#ifndef GAIT_MASTER_HPP
	#define GAIT_MASTER_HPP

	#define _USE_MATH_DEFINES //for pi (M_PI)

	#include "rclcpp/rclcpp.hpp" //ros 2 library
	#include <Eigen/Dense> //basic matrix and vector functions
	#include <math.h> //library for math functions
	#include <chrono> //timer control functionality
	#include "periaxis_interfaces/msg/leg_goal_coord.hpp" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
	#include "periaxis_interfaces/srv/get_position.hpp" //service file for dynamixel get position service request; generated from srv/GetPosition.srv
	#include "periaxis_interfaces/msg/gait_parameters.hpp" //message file for updated gait parameters topic this node subscribes to; generated from msg/GaitParameters.msg
	
	//Setup type def aliases for clock and duration
	// using CLK = std::chrono::steady_clock;
	// using duration_milis = std::chrono::milliseconds;
	typedef std::chrono::steady_clock CLK;
	typedef std::chrono::duration<float, std::milli> duration_milis;

	class GaitMaster; // Forward declaration

// LegController represents an individual leg of the robot. Each instance maintains its own state but shares a common ROS2 Node and publisher from parent GaitMaster instance
	class LegController {
	public:
		LegController(
			std::shared_ptr<GaitMaster> gaitMaster_node,
			const rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr legGoalCoord_pub,
			const CLK::time_point &clock_master,
			int legNum,
			double legZAngle_rad,
			double legPhaseOffset_ratio,
			int legPhaseOffset_count
		);
		
		// Publish current leg goal to legGoalCoord_pub
		void publishLegGoal(const CLK::time_point &CLK_master, const bool &me_reInitialise);

	private:
		// ## PRIVATE CLASS VARIABLES ##
		// Store ref to parent node
		std::weak_ptr<GaitMaster> _gaitMaster_node;  //weak_ptr provides a non-owning reference to resource managed by std::shared_ptr, avoids circular dependency in two way reference between parent-child class instance relationship
		// Shared publisher (const since does not change)
		const rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr _legGoalCoord_pub;
		
		// Timing and configuration variables
		
		
		// ###KEY CLASS OBJECT VARIABLES ###
		//Leg number (used for differentiating sent messages), assigned to instance during init
		const int _legNum;
		const double _legZAngle_rad; //the world Z axis angle the trace function will be rotated by to be above this leg
		//clock timer used to track legs step rate
		CLK::time_point _CLK_stepStart;
		
		int _cyclePeriod_millis; //used to calculate current cycle progress based on previous cycle period after updated from GaitMaster params
		int _traceRadius;
		double _step_offsetFactor;
		double _push_ratio;
		double _traceSTART;
		double _traceEND;
		
		//Transformation matrix for trace frame offset for leg
		Eigen::Matrix3d _legOffset_rot;  // rotation matrix to align function with leg angle
		//homogeneous tranformation matrix represents a rotation FOLLOWED by a translation
		Eigen::Matrix4d _A_traceFrame; // final transformation matrix to be constructed; set to Identity matrix to make bottom row 0,0,0,1  
		
		//Track the current position in the circles cycle
		double _PI_current_trace; // = 0
		
		//Tracks current trace position coords in the initial base frame (about 0,0,0); and in the transformed frame above the legs
		Eigen::Vector4d _traceCoord_base, _traceCoord_trans;
	
		// Forward declaration of functions
		void initialiseLegParameters(const double& currentCycleProgress);
	};	
	
	
// GaitMaster class inherits and represents the ROS2 Node gait_controller, class configures and manages a number of child instances of the LegController class for each leg of the robot
	class GaitMaster : public rclcpp::Node { //, public std::enable_shared_from_this<GaitMaster> { // [NOTE enable_shared_from_this is already inhereted from rclcpp::Node] also inherit from std::enable_shared_from_this<GaitMaster> to use get shared pointer durining init for LegController instances to reference
	public:
		GaitMaster();

		//Call this AFTER the node is created so a valid std::make_shared<GaitMaster>() can be passed to legController instances
		void initialiseLegInstances();
		
		// ## GETTER FUNCTIONS to expose class instance private variables as read-only ##
		inline int get_cyclePeriod_millis() const { return _cyclePeriod_millis; }
		inline int get_traceRadius() const { return _traceRadius; }
		inline double get_step_offsetFactor() const { return _step_offsetFactor; }
		inline double get_push_ratio() const { return _push_ratio; }
		inline double get_traceSTART() const { return _traceSTART; }
		inline double get_traceEND() const { return _traceEND; }
		//return const& reference to existing object rather than a copy
		const Eigen::Vector3d& get_traceOffset_trans() const { return _traceOffset_trans; }
		const Eigen::Matrix3d& get_traceOffset_rot() const { return _traceOffset_rot; }

	private:
		// ## PRIVATE CLASS FUNCTIONS ##
		// Timer-driven publishing loop
		void updateGaits();

		// Callback function for dynamic parameter updates
		void parameterCallback(const periaxis_interfaces::msg::GaitParameters::SharedPtr &msg);

		// Setter functions for updating gait parameters
		bool set_publishRate_ms(const int newValue);
		bool set_cyclePeriod_millis(const int newValue);
		bool set_traceRadius(const int newValue);
		bool set_step_offsetFactor(const double newValue);
		bool set_push_ratio(const double newValue);
		bool set_traceOffset_trans(const Eigen::Vector3d& new_translation);
		bool set_traceOffset_rot(const Eigen::Quaterniond& q);

		// ## PRIVATE CLASS VARIABLES ##
		// RANGE LIMITS FOR GAIT PARAMETERS
		static const int _cyclePeriod_millis_MIN;// = 1000;
		//[defined in gait_master.cpp] OR static constexpr double _cyclePeriod_millis_MIN = 1000;//constexpr to def in .hpp
		static const int _cyclePeriod_millis_MAX;// = 60000;
		static const int _traceRadius_MIN;// = 10;
		static const int _traceRadius_MAX;// = 150;
		static const double _step_offsetFactor_MIN;// = 1;
		static const double _step_offsetFactor_MAX;// = 50;
		static const double _push_ratio_MIN;// = 0.1;
		static const double _push_ratio_MAX;// = 0.9;
		// master trace translation limits
		static const double _x_transRangeLimit_abs;// = 80;
		static const double _y_transRangeLimit_abs;// = 80;
		static const double _z_transRangeLimit_MIN;// = 50;
		static const double _z_transRangeLimit_MAX;// = 200;
		
		// ROS 2 components
		// Vector holding all child LegController instances
		std::vector<std::unique_ptr<LegController>> _legControllers; //unique_ptr used to ensure GaitMaster owns its created instances of LegController, and are destroyed together with it
		rclcpp::TimerBase::SharedPtr _publish_timer; // Timer for periodic publishing
		// ROS 2 publisher shared across all child instances of LegController
		rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr _legGoalCoord_pub;
		// ROS 2 subscription to update gait_parameters
		rclcpp::Subscription<periaxis_interfaces::msg::GaitParameters>::SharedPtr _gaitParameters_sub;

		//Setup master clock pointer
		CLK::time_point _CLK_master;

		bool _reInitialiseFlag; //this controls when the leg class objects need to update their internal transforms and parameters after they have been updated from recieved periaxis_interfaces/gaitParameters message

		// GAIT PARAMETERS
		const int _leg_count;// = 5;
		int _publishRate_ms;// = 500;
		int _cyclePeriod_millis;// = 3500; //Period in ms for the complete PUSH & STEP cycle of a leg
		int _traceRadius;// = 5; //Radius in cm of the circle which is traced by the legs
		double _step_offsetFactor;// = 10; //adjusts tip of the leg +Y and -Z axis offset from trace path during STEP portion of cycle, higher is larger step height
		double _push_ratio;// = 0.6; //ratio leg should be pushing vs stepping (eg (2/5) as 40% stepping, 60% pushing)
		/* Start and end of (1/leg_count)*2*M_PI trace to have the trace path centred over y axis and equal for all legs to complete full circle
		 For 5 legs each legs trace path is (2/5)Pi; to get the first trace midway across the y-axis the range is, eg:
		 FROM (1/2)M_PI - 50%*(1/5)*2*M_PI = (3/10)M_PI = 0.942 rad
		 TO (1/2)M_PI + 50%*(1/5)*2*M_PI = (7/10) M_PI = 2.199 rad */
		double _traceSTART;// = (3/10)*M_PI;
		double _traceEND;// = (7/10)*M_PI;

		// MASTER TRACE TRANSFORMS, translation and rotation transforms determine the final position and orientation of the trace path function above the legs
		Eigen::Vector3d _traceOffset_trans;// = {0, 0, 15}; //translation offset {x,y,z} in cm from base frame [+z up, +y pointing to leg 0] where the trace function is drawn      
		Eigen::Matrix3d _traceOffset_rot;// = Eigen::Matrix3d::Identity(); //rotation matrix component of trace frame transform (init to no rotation)

	};

	
#endif // GAIT_MASTER_HPP