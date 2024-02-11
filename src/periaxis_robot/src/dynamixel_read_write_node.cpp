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
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "periaxis_robot/GetPosition.h"
#include "periaxis_robot/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

bool getPresentPositionCallback(
  periaxis_robot::GetPosition::Request & req,
  periaxis_robot::GetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", req.id, position);
    res.position = position;
    return true;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return false;
  }
}


void setPositionCallback(const periaxis_robot::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position1 = (unsigned int)msg->position1; // Convert int32 -> uint32
  uint32_t position2 = (unsigned int)msg->position2; // Convert int32 -> uint32
  uint32_t position3 = (unsigned int)msg->position3; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  //Set servo 1 position
  if(msg->arm1 == true) {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (uint8_t)msg->id1, ADDR_GOAL_POSITION, position1, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }

  //Set servo 2 position
  if(msg->arm2 == true) {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (uint8_t)msg->id2, ADDR_GOAL_POSITION, position2, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }

  //Set servo 3 position
  if(msg->arm3 == true) {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (uint8_t)msg->id3, ADDR_GOAL_POSITION, position3, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id3, msg->position3);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }

}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);  

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port! Check USB connection and port read/write permissions.");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  
  // Enable torque for the specified dynamixel
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }


  ros::init(argc, argv, "DMXL_read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer get_position_srv = nh.advertiseService("dynamixel/get_position", getPresentPositionCallback);
  ros::Subscriber set_position_sub = nh.subscribe("dynamixel/set_position", 1000, setPositionCallback);
  ros::spin();

  portHandler->closePort();
  return 0;
}
