cmake_minimum_required(VERSION 2.8.3)
project(neo_kinematics_mecanum)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
 nav_msgs
 roscpp
 #rostest
 tf 
 std_msgs
 sensor_msgs
 trajectory_msgs
 )

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  DIRECTORY msg
#  FILES EmergencyStopState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES std_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS std_msgs tf nav_msgs sensor_msgs
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/Mecanum4WKinematics.cpp)

add_executable(neo_kinematics_mecanum_node ros/src/neo_kinematics_mecanum_node.cpp)


add_executable(neo_kinematics_mecanum_sim_node ros/src/neo_kinematics_mecanum_sim_node.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(neo_kinematics_mecanum_node ${catkin_EXPORTED_TARGETS})
add_dependencies(neo_kinematics_mecanum_sim_node ${catkin_EXPORTED_TARGETS})


target_link_libraries(neo_kinematics_mecanum_node ${PROJECT_NAME})
target_link_libraries(neo_kinematics_mecanum_node ${catkin_LIBRARIES})

target_link_libraries(neo_kinematics_mecanum_sim_node ${PROJECT_NAME})
target_link_libraries(neo_kinematics_mecanum_sim_node ${catkin_LIBRARIES})

#target_link_libraries(neo_platformctrl_diff_node thread)


#############
## Install ##
#############

## Mark executables and/or libraries for installation

