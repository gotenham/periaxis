cmake_minimum_required(VERSION 3.5)
project(periaxis_node)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the ament packages and dependencies
find_package(ament_cmake REQUIRED) # new build engine for ROS 2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # more std message types for coordinates, quarternion etc
find_package(periaxis_interfaces REQUIRED) # Custom periaxis msg, srv etc. interface types
find_package(dynamixel_sdk REQUIRED) # Leg servo serial comms interface
find_package(Eigen3 REQUIRED) # dependant linear algebra library for periaxis_node

# ROS 2 launch functionality
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)

# find_package(rosidl_default_generators REQUIRED)  # To generate messages and services within node

include_directories(
  ${Eigen3_INCLUDE_DIRS}
)

# Link generated interfaces (msg, srv definitions) from periaxis_robot
# rosidl_get_typesupport_target(cpp_typesupport_target periaxis_robot "rosidl_typesupport_cpp")
# target_link_libraries(kinematic_solver "${cpp_typesupport_target}")
# target_link_libraries(gait_controller "${cpp_typesupport_target}")
# target_link_libraries(DMXL_read_write_node "${cpp_typesupport_target}")

# Add executable targets
add_executable(kinematic_solver src/leg_kinematicSolver.cpp)
ament_target_dependencies(kinematic_solver rclcpp sensor_msgs periaxis_interfaces)
#ament_target_dependencies(kinematic_solver rclcpp std_msgs geometry_msgs dynamixel_sdk periaxis_robot)
target_include_directories(kinematic_solver PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(kinematic_solver "${cpp_typesupport_target}")
# rosidl_target_interfaces(kinematic_solver ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gait_controller src/gait_controller.cpp)
ament_target_dependencies(gait_controller rclcpp periaxis_interfaces)
#ament_target_dependencies(gait_controller rclcpp std_msgs geometry_msgs dynamixel_sdk periaxis_robot)
target_include_directories(gait_controller PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(gait_controller "${cpp_typesupport_target}")
# rosidl_target_interfaces(gait_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(DMXL_read_write_node src/dynamixel_read_write_node.cpp)
ament_target_dependencies(DMXL_read_write_node rclcpp periaxis_interfaces dynamixel_sdk)
#ament_target_dependencies(DMXL_read_write_node rclcpp std_msgs dynamixel_sdk periaxis_robot)
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(DMXL_read_write_node "${cpp_typesupport_target}")
# rosidl_target_interfaces(DMXL_read_write_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install launch directory for package to workspace
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install targets
install(TARGETS
  kinematic_solver
  gait_controller
  DMXL_read_write_node
  DESTINATION lib/${PROJECT_NAME}
)

# Build as ROS 2 package
ament_package()
