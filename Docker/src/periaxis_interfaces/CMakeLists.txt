cmake_minimum_required(VERSION 3.5)
project(periaxis_interfaces)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
	# set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Find the ament packages and dependencies
find_package(ament_cmake REQUIRED) # new build engine for ROS 2
# find_package(rclcpp REQUIRED) # TBD May not be required within this CMakeLists.txt  seeing as interface generation file?
#find_package(rosidl_default_runtime REQUIRED) # TBD remove?
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # more std message types for coordinates, quarternion etc
find_package(rosidl_default_generators REQUIRED)  # To generate messages and services within node

#### ROS2 Interface Directives ####
# Specify interface generators
set(interface_files
  # Package Message Types
  "msg/LegGoalCoord.msg"
  "msg/SetPosition.msg"
  "msg/GaitParameters.msg"
  # Package Service Types
  "srv/GetPosition.srv"
)
# Generate interface files from 'interface_files' list
rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES sensor_msgs geometry_msgs
)
# install(DIRECTORY include/${PROJECT_NAME}/
  # DESTINATION include/${PROJECT_NAME}
# )

# install(
  # DIRECTORY msg srv
  # DESTINATION share/${PROJECT_NAME}
# )

#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(periaxis_robot "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
#### [END] ROS2 Interface Directives ####

# Build as ROS 2 package
ament_package()
