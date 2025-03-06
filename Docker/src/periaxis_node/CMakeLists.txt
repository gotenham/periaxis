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
#find_package(Eigen3 REQUIRED NO_MODULE) #tells CMake to use the system-installed version instead of looking for FindEigen3.cmake

# include_directories(
	# ${Eigen3_INCLUDE_DIRS}
# )
include_directories(
	/usr/include/eigen3 # ${Eigen3_INCLUDE_DIRS}
)

# Add executable targets
add_executable(kinematic_solver src/leg_kinematicSolver.cpp)
ament_target_dependencies(kinematic_solver rclcpp sensor_msgs periaxis_interfaces)
target_include_directories(kinematic_solver PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
   # "${Eigen3_INCLUDE_DIRS}")


add_executable(gait_controller_node src/gait_controller_node.cpp src/gait_master.cpp)
ament_target_dependencies(gait_controller_node rclcpp periaxis_interfaces Eigen3)
target_include_directories(gait_controller_node PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
   # "${Eigen3_INCLUDE_DIRS}")

add_executable(DMXL_read_write_node src/dynamixel_read_write_node.cpp)
ament_target_dependencies(DMXL_read_write_node rclcpp periaxis_interfaces dynamixel_sdk)

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
  gait_controller_node
  DMXL_read_write_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp periaxis_interfaces Eigen3)

# Build as ROS 2 package
ament_package()