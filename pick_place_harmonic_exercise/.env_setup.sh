#!/bin/bash
export GZ_VERSION=harmonic
source /opt/ros/humble/setup.bash
[ -d /home/ws/install ] && source /home/ws/install/setup.bash
GZ_CTRL=/opt/jderobot/IndustrialRobots/pick_place_harmonic_exercise/gz_ros2_control/install
[ -d $GZ_CTRL ] && source $GZ_CTRL/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_CTRL/gz_ros2_control/lib:/opt/ros/humble/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/ws/install/ur5_gripper_description/share:/home/ws/install/robotiq_description/share:$GZ_SIM_RESOURCE_PATH
export LD_LIBRARY_PATH=$GZ_CTRL/gz_ros2_control/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
echo "Environment ready for Pick Place Harmonic - Using custom gz_ros2_control"
