# Pick and Place Harmonic Exercise - Installation Guide

## Overview
UR5 + Robotiq gripper setup for Pick and Place exercise with Gazebo Harmonic and MoveIt2.

## Packages Included
- `ur5_gripper_description` - Robot URDF, launch files, controllers
- `ur5_gripper_moveit_config` - MoveIt2 planning configuration
- `robotiq_description` - Robotiq 85 gripper models and warehouse world
- `pick_place_harmonic` - ROS2 exercise package

## Prerequisites
- ROS2 Humble
- Gazebo Harmonic (gz-sim)
- MoveIt2
- **gz_ros2_control** (install separately)

## Installing gz_ros2_control

### Option 1: Install from apt (recommended)
```bash
sudo apt install ros-humble-gz-ros2-control
```

### Option 2: Build from source
```bash
cd ~/dev_ws/src
git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
cd ~/dev_ws
colcon build --packages-select gz_ros2_control
```

## Building the Exercise Packages

```bash
# Clone into your workspace
cd ~/dev_ws/src
git clone https://github.com/JdeRobot/IndustrialRobots.git

# Build the packages
cd ~/dev_ws
colcon build --packages-select \
    ur5_gripper_description \
    ur5_gripper_moveit_config \
    robotiq_description \
    pick_place_harmonic

# Source the workspace
source install/setup.bash
```

## Environment Setup
Set Gazebo Harmonic environment variables (defined in RoboticsApplicationManager):
```bash
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="/path/to/gz_ros2_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="/path/to/packages/share:${GZ_SIM_RESOURCE_PATH}"
```

## Usage

### With JdeRobot Academy
The exercise launches automatically through the Academy interface.

### Standalone Testing
```bash
source install/setup.bash
source pick_place_harmonic_exercise/.env_setup.sh
ros2 launch ur5_gripper_description spawn_robot_warehouse.launch.py
```

## Features
- Gazebo Harmonic simulation
- MoveIt2 motion planning with collision avoidance
- Pick and place of colored objects in warehouse environment
