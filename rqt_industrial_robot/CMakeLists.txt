cmake_minimum_required(VERSION 3.0.2)
project(rqt_industrial_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  gazebo_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS 
  scripts/rqt_kinematics
  scripts/rqt_vacuum_gripper
  scripts/rqt_camera
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin_kinematics.xml
  plugin_vacuum_gripper.xml
  plugin_camera.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
