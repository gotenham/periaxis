//  This C++ code manages a node which calculates the current coordinate of the point which the tip of each leg traces along the trace function
#include "rclcpp/rclcpp.hpp" //ros 2 library
#include "periaxis_node/gait_master.hpp"

int main(int argc, char** argv) {
    //Init ROS2
	rclcpp::init(argc, argv);
	
	auto gaitMaster = std::make_shared<GaitMaster>();
	gaitMaster->initialiseLegInstances(); // Critical: Call after construction!
	// Create class instance as node and spin
    rclcpp::spin(gaitMaster);
	
	//Shutdown ROS2 node
    rclcpp::shutdown();
    return 0;
}

//END NODE.cpp





