cmake_minimum_required(VERSION 3.0.2)
project(rqt_industrial_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  gazebo_msgs
  geometry_msgs
  trajectory_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/rqt_kinematics.py
  scripts/rqt_vacuum_gripper.py
  scripts/rqt_camera.py
  scripts/rqt_mobile_manipulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resources
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  all.perspective
  kinematics.perspective
  mobile_manipulator.perspective
  plugin_camera.xml
  plugin_kinematics.xml
  plugin_mobile_manipulator.xml
  plugin_vacuum_gripper.xml
  ur5_vacuum_gripper.perspective
  ur5_vacuum_gripper_camera.perspective
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
