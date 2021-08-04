# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(ur5_gripper_demo)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation)
# Declare our catkin package.

#add_message files
add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
    scripts/ur5_gripper.py
    scripts/ur5_gripper_test.py
    scripts/ur5_pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch models rviz worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
