#ifndef GAIT_CONTROLLER_HPP
	#define GAIT_CONTROLLER_HPP

	#define PARENT_NODE_NAME "gait_controller"
	#define _USE_MATH_DEFINES //for pi (M_PI)

	#include "rclcpp/rclcpp.hpp" //ros 2 library
	#include <Eigen/Dense> //basic matrix and vector functions
	#include <math.h> //library for math functions
	#include <chrono> //timer control functionality
	#include "periaxis_interfaces/msg/leg_goal_coord.hpp" //message file for subscribed leg goal topic message; generated from msg/legGoalCoord.msg
	#include "periaxis_interfaces/srv/get_position.hpp" //service file for dynamixel get position service request; generated from srv/GetPosition.srv
	#include "periaxis_interfaces/msg/gait_parameters.hpp" //message file for updated gait parameters topic this node subscribes to; generated from msg/GaitParameters.msg

	//Setup type def aliases for clock and milliseconds
	typedef std::chrono::high_resolution_clock CLK;
	typedef std::chrono::duration<float, std::milli> duration_milis;

	class LegController {
	public:
		using CLK = std::chrono::steady_clock;
		using duration_milis = std::chrono::milliseconds;

		LegController(
			std::shared_ptr<GaitMaster> gaitMaster_node,
			const rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr legGoalCoord_pub,
			const CLK::time_point &clock_master,
			int legNum,
			double legZAngle_rad,
			double legPhaseOffset_ratio,
			double legPhaseOffset_count
		);

		void publishLegGoal(const CLK::time_point &CLK_master, const bool &me_reInitialise);

	private:
		// Weak pointer to parent node
		std::weak_ptr<GaitMaster> _gaitMaster_node;
		
		// Publisher for leg goal coordinates
		const rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr _legGoalCoord_pub;
		
		// Timing and configuration variables
		CLK::time_point _CLK_stepStart;
		const int _legNum;
		const double _legZAngle_rad;
		
		int _cyclePeriod_millis;
		int _traceRadius;
		double _step_offsetFactor;
		double _push_ratio;
		double _traceSTART;
		double _traceEND;
		
		Eigen::Matrix3d _legOffset_rot;
		Eigen::Matrix4d _A_traceFrame = Eigen::Matrix4d::Identity();
		
		double _PI_current_trace = 0;
		Eigen::Vector4d _traceCoord_base, _traceCoord_trans;

		void initialiseLegParameters(const double& currentCycleProgress);
	};	
		
	class GaitMaster : public rclcpp::Node { //, public std::enable_shared_from_this<GaitMaster>
	public:
		GaitMaster();

		// Getter functions for private class variables
		inline int get_cyclePeriod_millis() const { return _cyclePeriod_millis; }
		inline int get_traceRadius() const { return _traceRadius; }
		inline double get_step_offsetFactor() const { return _step_offsetFactor; }
		inline double get_push_ratio() const { return _push_ratio; }
		inline double get_traceSTART() const { return _traceSTART; }
		inline double get_traceEND() const { return _traceEND; }
		const Eigen::Vector3d& get_traceOffset_trans() const { return _traceOffset_trans; }
		const Eigen::Matrix3d& get_traceOffset_rot() const { return _traceOffset_rot; }

	private:
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

		// ROS 2 components
		std::vector<std::unique_ptr<LegController>> _legControllers;
		rclcpp::TimerBase::SharedPtr _publish_timer;
		rclcpp::Publisher<periaxis_interfaces::msg::LegGoalCoord>::SharedPtr _legGoalCoord_pub;
		rclcpp::Subscription<periaxis_interfaces::msg::GaitParameters>::SharedPtr _gaitParameters_sub;

		// Master clock reference
		CLK::time_point _CLK_master = CLK::now();
		bool _reInitialiseFlag = false;

		// Gait parameters
		int _leg_count = 5;
		int _publishRate_ms = 500;
		int _cyclePeriod_millis = 3500;
		int _traceRadius = 5;
		double _step_offsetFactor = 10;
		double _push_ratio = 0.6;
		const double _traceSTART = (3.0 / 10.0) * M_PI;
		const double _traceEND = (7.0 / 10.0) * M_PI;

		// Master trace transforms
		Eigen::Vector3d _traceOffset_trans = {0, 0, 15};
		Eigen::Matrix3d _traceOffset_rot = Eigen::Matrix3d::Identity();

		// Parameter limits
		static const int _cyclePeriod_millis_MIN = 2000;
		static const int _cyclePeriod_millis_MAX = 20000;
		static const int _traceRadius_MIN = 1;
		static const int _traceRadius_MAX = 15;
		static const double _step_offsetFactor_MIN = 1;
		static const double _step_offsetFactor_MAX = 50;
		static const double _push_ratio_MIN = 0.1;
		static const double _push_ratio_MAX = 0.9;
		static const double _x_transRangeLimit_abs = 8;
		static const double _y_transRangeLimit_abs = 8;
		static const double _z_transRangeLimit_MIN = 5;
		static const double _z_transRangeLimit_MAX = 20;
	};

	
#endif // GAIT_CONTROLLER_HPP