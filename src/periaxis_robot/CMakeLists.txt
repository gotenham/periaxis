cmake_minimum_required(VERSION 2.8.3)
project(periaxis_robot)

	## Compile as C++11, supported in ROS Kinetic and newer
	# add_compile_options(-std=c++11)

	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_msgs
    geometry_msgs
		message_generation #used to generate our header file from .srv
		dynamixel_sdk
	)
	## System dependencies are found with CMake's conventions
	# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
   legGoalCoord.msg #defines the mesage type used for leg goal published topic
   SetPosition.msg #message type to send command topics for dynamixel servo control
   gaitParameters.msg #for updating the gait and trace parameters on the fly
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  matrix_3x3.srv #left in from previous project, service type for recieving a rotation matrix and converting it to a quartention representation
  GetPosition.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # define the data type dependancies to generate the header from our srv or msg type
  geometry_msgs # point and quartention standard types are used in gaitParameters custom msg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hbk
  CATKIN_DEPENDS 
  roscpp 
  message_runtime #allows compiling of message and service data types at runtime
  dynamixel_sdk
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include ${catkin_INCLUDE_DIRS}
)

#add dependancy for generated header and library file incl custom service and msg types
add_executable(invKinematicSolver src/leg_kinematicSolver.cpp)
add_dependencies(invKinematicSolver ${catkin_EXPORTED_TARGETS})
add_dependencies(invKinematicSolver ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(invKinematicSolver ${catkin_LIBRARIES})

add_executable(gait_controller src/gait_controller.cpp)
add_dependencies(gait_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(gait_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(gait_controller ${catkin_LIBRARIES})

add_executable(DMXL_read_write_node src/dynamixel_read_write_node.cpp)
add_dependencies(DMXL_read_write_node ${catkin_EXPORTED_TARGETS})
add_dependencies(DMXL_read_write_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(DMXL_read_write_node ${catkin_LIBRARIES})

#Install packages so they are findable from rosrun, roslaunch
#http://docs.ros.org/en/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS invKinematicSolver gait_controller DMXL_read_write_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



