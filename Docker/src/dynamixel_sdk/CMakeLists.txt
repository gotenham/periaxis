################################################################################
# Set minimum required version of cmake, project name, and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

# Default to C++14
#set(CMAKE_CXX_STANDARD 14)
# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################
# Find ament packages and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_export_include_directories(include)
ament_export_libraries(dynamixel_sdk)
ament_export_dependencies(rclcpp rclpy)

################################################################################
# Build
################################################################################
include_directories(
  include/dynamixel_sdk
)

if(APPLE)
  add_library(dynamixel_sdk SHARED
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
  )
else()
  add_library(dynamixel_sdk SHARED
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

target_include_directories(dynamixel_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dynamixel_sdk)

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_sdk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  src/${PROJECT_NAME}/group_bulk_read.py
  src/${PROJECT_NAME}/group_bulk_write.py
  src/${PROJECT_NAME}/group_sync_read.py
  src/${PROJECT_NAME}/group_sync_write.py
  src/${PROJECT_NAME}/packet_handler.py
  src/${PROJECT_NAME}/port_handler.py
  src/${PROJECT_NAME}/protocol1_packet_handler.py
  src/${PROJECT_NAME}/protocol2_packet_handler.py
  src/${PROJECT_NAME}/robotis_def.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Export package
################################################################################
ament_package()
