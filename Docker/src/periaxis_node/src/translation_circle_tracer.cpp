#include "rclcpp/rclcpp.hpp" //ros 2 library

//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Quaternion.h>
#include <chrono>
#include <cmath>
#include "periaxis_interfaces/msg/gait_parameters.hpp" //message file for updated gait parameters topic this node publishes to

int main(int argc, char** argv) {
	//Initialise ROS 2 node
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("translation_circle_tracer"); //Start node 

    // Define parameters with defaults
    double radius = 5;                  // Default trace radius cm
    double cycle_period = 10000;          // Default period in ms
    double z_offset = 17;               // Default Z offset cm
    double publish_frequency = cycle_period / 100; //50.0;     // publish frequency Hz

    // Publisher
    //ros::Publisher gait_pub = nh.advertise<periaxis_interfaces::gaitParameters>("periaxis_robot/gaitParameters", 10);
	auto gait_pub = nh->create_publisher<periaxis_interfaces::msg::GaitParameters>("periaxis_robot/gaitParameters", 10); //ROS2
    //ros::Subscriber sub = nh.subscribe("periaxis_robot/gaitParameters", 10, ParameterUpdate);

    // Derived values
    double angular_velocity = 2 * M_PI / (cycle_period / 1000.0); // radians per second
	// Configure node message publishing rate
	rclcpp::Rate rate(publish_frequency); // rate in Hz

    // Timing using chrono
    auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        // Compute elapsed time
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();

        // Calculate current angle
        double angle = angular_velocity * elapsed_time;

        // Create and populate gaitParameters message
        periaxis_interfaces::msg::GaitParameters gait_param_msg;
		
        gait_param_msg.arm_live_update = true; // Set the message as armed (ie flag the new parameters for execution)

        // Update traceOffset_trans
        gait_param_msg.trace_offset_trans.x = radius * cos(angle);
        gait_param_msg.trace_offset_trans.y = 0.0; // Currently set to always zero in XZ plane
        gait_param_msg.trace_offset_trans.z = radius * sin(angle) + z_offset;

        // Example values for other fields, not currently implemented
		//gait_param_msg.cycle_period_millis = 5000; //default walking circle trace cycle period
        //gait_param_msg.trace_radius = 10; //default walking circle trace cycle period
        //gait_param_msg.trace_offset_rot.x = 0.0; // Default, no trace frame rotation
        //gait_param_msg.trace_offset_rot.y = 0.0;
        //gait_param_msg.trace_offset_rot.z = 0.0;
        //gait_param_msg.trace_offset_rot.w = 1.0;
        //gait_param_msg.push_ratio = 0.6;          // Default leg cycle push ratio
        //gait_param_msg.step_offset_factor = 10.0;  // Default leg stepping offset factor from trace frame

        // Publish the message
        gait_pub->publish(gait_param_msg);

        // Log the message to ROS node as published
		RCLCPP_INFO(nh->get_logger(), "Publishing Gait Parameters: [trace_offset_trans: x: %f, y: %f, z: %f]",
                 gait_param_msg.trace_offset_trans.x, gait_param_msg.trace_offset_trans.y, gait_param_msg.trace_offset_trans.z);
					
        // Spin and sleep
        rclcpp::spin_some(nh); //ROS2 process node subscriptions and service callbacks (Non on node at current point in design)
        rate.sleep(); // Sleep to maintain loop publishing rate
		
    }
	
	// Shutdown ROS 2 node
	rclcpp::shutdown();
    return 0;
}

/* NOTES ON ADDING NEW cpp FILE AS NODE TO ROS PROJECT
1. Save the new .cpp file in src directory of relevent ROS package.

2. Update ROS project CMakeLists.txt
Ensure all dependencies are found, including geometry_msgs and custom message package, if they’re not already included:
	find_package(catkin REQUIRED COMPONENTS
		roscpp
		geometry_msgs
		custom_msgs
	)
Add the Executable: Add .cpp file as an executable and link necessary libraries:
	add_executable(circle_tracer_node src/circle_tracer_node.cpp)
	target_link_libraries(circle_tracer_node ${catkin_LIBRARIES})
	add_dependencies(circle_tracer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

3. Update package.xml with build and run dependencies if not already configured:
Add Build Dependencies:
	<build_depend>roscpp</build_depend>
	<build_depend>geometry_msgs</build_depend>
	<build_depend>gaitParameters</build_depend>
Add Run Dependencies:
	<exec_depend>roscpp</exec_depend>
	<exec_depend>geometry_msgs</exec_depend>
	<exec_depend>gaitParameters</exec_depend>

4. Build the package from a terminal in the ROS workspace folder for the project:
	catkin_make
Source the updated workspace to the terminal:
	source devel/setup.bash

5. Run the Node:
	rosrun periaxis_robot translation_circle_tracer

*/