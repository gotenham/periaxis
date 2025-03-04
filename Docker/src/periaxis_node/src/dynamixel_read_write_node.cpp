// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
 ****************
 * Edited @Dec 2024 to port functionality from ROS 1 to ROS 2 framework
*******************************************************************************/

#include "rclcpp/rclcpp.hpp" //ros 2 library

//#include "std_msgs/msg/string.hpp" // ros 2 std msg type
#include "periaxis_interfaces/srv/get_position.hpp"
#include "periaxis_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Control table memory size (in bytes)
#define LEN_GOAL_POSITION     2

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// PortHandler * _portHandler;
// PacketHandler * _packetHandler;

// GroupBulkWrite * _groupBulkWrite;

// //Node logger reference
// rclcpp::Logger node_logger; //ROS2

// Define custom log levels (TEMP SOLUTION, BETTER WAYS TO IMPLEMENT LOGGING)
// enum ROSLoggerLevel {
    // LOG_INFO = 0,
	// LOG_DEBUG,
    // LOG_WARN,
    // LOG_ERROR,
    // LOG_FATAL
// };

//Log messages to ROS2 node using ostringstream, log_msg is cleared after sending log so it can be reused by calling function
// void logStreamToNode(const rclcpp::Logger &logger, const ROSLoggerLevel log_level, std::ostringstream &log_msg) {
	// /*ROS2 Logging Options:
		// RCLCPP_INFO //Log high-level information, system startup, state transitions etc
		// RCLCPP_DEBUG //detailed information useful for debugging
		// RCLCPP_WARN //Log unexpected conditions, deprecated features, or non-critical errors that don’t necessarily halt the system.
		// RCLCPP_ERROR //Log recoverable errors where action should be taken, such as missing params, coms errors, or issues that prevent normal operation.
		// RCLCPP_FATAL //Log catastrophic failures, such as inability to access essential resources; often followed by an exit or shutdown of the system.
	// */
    // // Log the message based on the log level
    // switch (log_level) {
        // case LOG_INFO:
            // RCLCPP_INFO(logger, "%s", log_msg.str().c_str());
			// // RCLCPP_INFO_STREAM(node->get_logger(), log_msg);
            // break;
		// case LOG_DEBUG:
            // RCLCPP_DEBUG(logger, "%s", log_msg.str().c_str());
			// // RCLCPP_DEBUG_STREAM(node->get_logger(), log_msg);
            // break;
        // case LOG_WARN:
            // RCLCPP_WARN(logger, "%s", log_msg.str().c_str());
			// // RCLCPP_WARN_STREAM(node->get_logger(), log_msg);
            // break;
        // case LOG_ERROR:
            // RCLCPP_ERROR(logger, "%s", log_msg.str().c_str());
			// // RCLCPP_ERROR_STREAM(node->get_logger(), log_msg);
            // break;
        // case LOG_FATAL:
            // RCLCPP_FATAL(logger, "%s", log_msg.str().c_str());
			// // RCLCPP_FATAL_STREAM(node->get_logger(), log_msg);
            // break;
        // default: // Default to INFO if no valid log level
            // RCLCPP_INFO(logger, "%s", log_msg.str().c_str());
            // break;
    // }
    // // Clear the string buffer of the ostringstream and reset the state so variable can be reused in calling function
    // log_msg.str("");   // Clear content of stream
    // log_msg.clear();   // Reset the stream's state (ie to clear errors, if any)
// }


// START CLASS DEF ///////////////////////////////////////////////////////////////////
class DMXLReadWriteNode : public rclcpp::Node {
	public:
		// CLASS CONSTRUCTOR
		DMXLReadWriteNode() : Node("DMXL_read_write_node")
		{
			//Starting ROS 2 Node log message
			RCLCPP_INFO(this->get_logger(), "Node starting...");
			
			// Init serial comms components
			try {
				// Configure ports and packet handler for serial comms to servos
				_portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
				_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

				// Open port
				if (!_portHandler->openPort()) {
					RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICE_NAME); //ROS 2 log
					throw std::runtime_error("Failed to open the port! Check USB connection and port read/write permissions.");
				} else {
					_port_open = true; // flag that the class instance has an open port
				}
				// Set baudrate
				if (!_portHandler->setBaudRate(BAUDRATE)) {
					throw std::runtime_error("Failed to set the baudrate!");
				}

				// Init GroupBulkWrite instance <<TBD check the usage of this
				_groupBulkWrite = std::make_unique<dynamixel::GroupBulkWrite>(_portHandler, _packetHandler);

			} catch (const std::exception & e) {
				RCLCPP_FATAL(this->get_logger(), "Error initialising serial communication: %s", e.what());
				rclcpp::shutdown(); // Shutdown ROS 2 in case of failure
				return; //node startup failed
			}

			// Create service for getting servo current position
			_get_position_srv = this->create_service<periaxis_interfaces::srv::GetPosition>(
				"dynamixel/get_position", 
				[this](const std::shared_ptr<periaxis_interfaces::srv::GetPosition::Request> req,
					   std::shared_ptr<periaxis_interfaces::srv::GetPosition::Response> res) {
					this->getPresentPositionCallback(req, res);
				}
			);
			// _get_position_srv = this->create_service<periaxis_interfaces::srv::GetPosition>(
				// "dynamixel/get_position", std::bind(&DMXLReadWriteNode::getPresentPositionCallback, this, std::placeholders::_1, std::placeholders::_2)
			// );

			// Create subscription for setting leg position (i.e. 3 servos per leg)
			_set_position_sub = this->create_subscription<periaxis_interfaces::msg::SetPosition>(
				"dynamixel/set_position", 5, 
				[this](const std::shared_ptr<periaxis_interfaces::msg::SetPosition> msg) {
					this->setPositionCallback(msg);
				}
			);
			// _set_position_sub = this->create_subscription<periaxis_interfaces::msg::SetPosition>(
				// "dynamixel/set_position", 5, std::bind(&DMXLReadWriteNode::setPositionCallback, this, std::placeholders::_1)
			// );
			
			RCLCPP_INFO(this->get_logger(), "Successfully initialised and running!"); //ROS 2 log
		}
		
		// CLASS DECONSTRUCTOR
		~DMXLReadWriteNode()
		{
			// Close the serial port if it was opened successfully
			if (_portHandler != nullptr && _port_open) {
				_portHandler->closePort();
				RCLCPP_INFO(this->get_logger(), "Serial port closed.");
			}
		}

	private:
		// PRIVATE CLASS COMPONENTS
		// Declare service, subscription components as class private members
		rclcpp::Service<periaxis_interfaces::srv::GetPosition>::SharedPtr _get_position_srv;
		rclcpp::Subscription<periaxis_interfaces::msg::SetPosition>::SharedPtr _set_position_sub;
		// Dynamixel communication components
		dynamixel::PortHandler* _portHandler;
		dynamixel::PacketHandler* _packetHandler;
		std::unique_ptr<dynamixel::GroupBulkWrite> _groupBulkWrite; //unique_ptr used for auto memory cleanup on node out of scope
		bool _port_open = false; // Track if port is open for deconstructor
		// NEXT SECTION

		// Callback for the service "dynamixel/get_position"
		void getPresentPositionCallback(
			const std::shared_ptr<periaxis_interfaces::srv::GetPosition::Request> req,
			std::shared_ptr<periaxis_interfaces::srv::GetPosition::Response> res) 
		{		
			// Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
			// Read Present Position (length : 4 bytes) and Convert uint32 -> int32
			// When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.

			//auto node_logger = rclcpp::get_logger("DMXL_read_write_node"); //TEMP SOLUTION, BETTER TO CREATE NODE INHERITING CLASS TO MANAGE THIS
			
			uint8_t dxl_error = 0;
			int dxl_comm_result = COMM_TX_FAIL;
			
			//NOTE: Servo position is uint16_t 0~1023 value for 0 to 300deg rotation, top position is 512 at 150deg; each unit is 0.29deg.
			uint16_t position1 = 0;
			uint16_t position2 = 0;
			uint16_t position3 = 0;
			
			//Get servo 1 position
			dxl_comm_result = _packetHandler->read2ByteTxRx(_portHandler, (uint8_t)req->id1, ADDR_PRESENT_POSITION, &position1, &dxl_error);
			if (dxl_comm_result == COMM_SUCCESS) {
				//ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req->id1, position1);
				//log_msg << "getPosition : [ID:%d] -> [POSITION:%d]", req->id1, position1;
				//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
				RCLCPP_INFO(this->get_logger(), "getPosition : [ID:%d] -> [POSITION:%d]", req->id1, position1); //send log to ROS2 node
				
				res->valid1 = true;
				res->position1 = position1;
			} else {
				//ROS_WARN_STREAM("Failed to get position servo %d! Result: %d", req->id1, dxl_comm_result);
				//log_msg << "Failed to get position of servo ID: %d! Result: %d", req->id1, dxl_comm_result;
				//logStreamToNode(node_logger, LOG_WARN, log_msg); //send log to ROS2 node
				RCLCPP_WARN(this->get_logger(), "Failed to get position of servo ID: %d! Result: %d", req->id1, dxl_comm_result); //send log to ROS2 node
				
				res->valid1 = false;
				res->position1 = 0; //default invalid value, not to be processed by calling function
			}
			
			//Get servo 2 position
			dxl_comm_result = _packetHandler->read2ByteTxRx(_portHandler, (uint8_t)req->id2, ADDR_PRESENT_POSITION, &position2, &dxl_error);
			if (dxl_comm_result == COMM_SUCCESS) {
				//ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req->id2, position2);
				//log_msg << "getPosition : [ID:%d] -> [POSITION:%d]", req->id2, position2;
				//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
				RCLCPP_INFO(this->get_logger(), "getPosition : [ID:%d] -> [POSITION:%d]", req->id2, position2); //send log to ROS2 node
				
				res->valid2 = true;
				res->position2 = position2;
			} else {
				//ROS_WARN_STREAM("Failed to get position servo %d! Result: %d", req->id2, dxl_comm_result);
				//log_msg << "Failed to get position of servo ID: %d! Result: %d", req->id2, dxl_comm_result;
				//logStreamToNode(node_logger, LOG_WARN, log_msg); //send log to ROS2 node
				RCLCPP_WARN(this->get_logger(), "Failed to get position of servo ID: %d! Result: %d", req->id2, dxl_comm_result); //send log to ROS2 node
				
				res->valid2 = false;
				res->position2 = 0; //default invalid value, not to be processed by calling function
			}
			
			//Get servo 3 position
			dxl_comm_result = _packetHandler->read2ByteTxRx(_portHandler, (uint8_t)req->id3, ADDR_PRESENT_POSITION, &position3, &dxl_error);
			if (dxl_comm_result == COMM_SUCCESS) {
				//ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req->id3, position3);
				//log_msg << "getPosition : [ID:%d] -> [POSITION:%d]", req->id3, position3;
				//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
				RCLCPP_INFO(this->get_logger(), "getPosition : [ID:%d] -> [POSITION:%d]", req->id3, position3); //send log to ROS2 node
				
				res->valid3 = true;
				res->position3 = position3;
			} else {
				//ROS_WARN_STREAM("Failed to get position servo %d! Result: %d", req->id3, dxl_comm_result);
				//log_msg << "Failed to get position of servo ID: %d! Result: %d", req->id3, dxl_comm_result;
				//logStreamToNode(node_logger, LOG_WARN, log_msg); //send log to ROS2 node
				RCLCPP_WARN(this->get_logger(), "Failed to get position of servo ID: %d! Result: %d", req->id3, dxl_comm_result); //send log to ROS2 node
				
				res->valid3 = false;
				res->position3 = 0; //default invalid value, not to be processed by calling function
			}
			
		}

		// Callback for the subscription to "dynamixel/set_position"
		void setPositionCallback(const std::shared_ptr<periaxis_interfaces::msg::SetPosition> &msg)
		{
			//uint8_t dxl_error = 0;
			int dxl_comm_result = COMM_TX_FAIL;
			//std::ostringstream log_msg; //reuseable string stream for logging to ROS2 node
			
			//auto node_logger = rclcpp::get_logger("DMXL_read_write_node"); //TEMP SOLUTION, BETTER TO CREATE NODE INHERITING CLASS TO MANAGE THIS
			
			// Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
			// Write Goal Position (length : 4 bytes)
			// When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.

			//Set servo 1 position
			if(msg->arm1 == true) {
				dxl_comm_result = _packetHandler->write2ByteTxOnly(_portHandler, (uint8_t)msg->id1, ADDR_GOAL_POSITION, msg->position1);
				if (dxl_comm_result == COMM_SUCCESS) {
					//ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
					//log_msg << "setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1;
					//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
					
					RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1); //send log to ROS2 node
				} else {
					//ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
					//log_msg << "Failed to set position of [ID:%d]! Result: %d", msg->id1, dxl_comm_result;
					//logStreamToNode(node_logger, LOG_ERROR, log_msg); //send log to ROS2 node
					
					RCLCPP_ERROR(this->get_logger(), "Failed to set position of [ID:%d]! Result: %d", msg->id1, dxl_comm_result); //send log to ROS2 node
				}
			}

			//Set servo 2 position
			if(msg->arm2 == true) {
				dxl_comm_result = _packetHandler->write2ByteTxOnly(_portHandler, (uint8_t)msg->id2, ADDR_GOAL_POSITION, msg->position2);
				if (dxl_comm_result == COMM_SUCCESS) {
					//ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
					//log_msg << "setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2;
					//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
					
					RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2); //send log to ROS2 node
				} else {
					//ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
					//log_msg << "Failed to set position of [ID:%d]! Result: %d", msg->id2, dxl_comm_result;
					//logStreamToNode(node_logger, LOG_ERROR, log_msg); //send log to ROS2 node
					
					RCLCPP_ERROR(this->get_logger(), "Failed to set position of [ID:%d]! Result: %d", msg->id2, dxl_comm_result); //send log to ROS2 node
				}
			}

			//Set servo 3 position
			if(msg->arm3 == true) {
				dxl_comm_result = _packetHandler->write2ByteTxOnly(_portHandler, (uint8_t)msg->id3, ADDR_GOAL_POSITION, msg->position3);
				if (dxl_comm_result == COMM_SUCCESS) {
					//ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3);
					//log_msg << "setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3;
					//logStreamToNode(node_logger, LOG_INFO, log_msg); //send log to ROS2 node
					
					RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3); //send log to ROS2 node
				} else {
					//ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
					//log_msg << "Failed to set position of [ID:%d]! Result: %d", msg->id3, dxl_comm_result;
					//logStreamToNode(node_logger, LOG_ERROR, log_msg); //send log to ROS2 node
					
					RCLCPP_ERROR(this->get_logger(), "Failed to set position of [ID:%d]! Result: %d", msg->id3, dxl_comm_result); //send log to ROS2 node
				}
			}
		}
};

// END CLASS DEF ///////////////////////////////////////////////////////////////////

		
int main(int argc, char ** argv)
{
	//Initialise ROS 2 node
	rclcpp::init(argc, argv);

	// Create a shared pointer to DMXLReadWriteNode instance
	auto node = std::make_shared<DMXLReadWriteNode>();
	
	//Spin ROS node to process service requests
	rclcpp::spin(node);

	// Shutdown ROS 2 node
	rclcpp::shutdown();
	
	return 0;
}
