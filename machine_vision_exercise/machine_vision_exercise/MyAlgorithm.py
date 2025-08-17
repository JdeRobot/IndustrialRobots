#!/usr/bin/env python3

"""
Simple test script for the JdeRobot Pick and Place exercise
Uses the HAL API to test basic robot movements
"""

import time
import sys

# Import the movement functions from HAL
try:
    from HAL import MoveAbsJ, MoveLinear, MoveJoint, GripperSet, MoveRelLinear, MoveSingleJ
    print("HAL API imported successfully!")
except ImportError as e:
    print(f"Error importing HAL: {e}")
    print("Make sure HAL.py is in your Python path")
    sys.exit(1)

def test_robot_movements():
    """Test basic robot movements using the HAL API"""
    
    print("=== Starting Robot Movement Tests ===")
    print()
    
    # Test 1: Move robot to home position using joint angles
    print("Test 1: Moving to home position...")
    home_joints = [0, 0, 0, 0, 0, 0]  # All joints to 0 degrees
    MoveAbsJ(home_joints, speed=0.5, wait_time=2.0)
    
    # Test 2: Move to a specific joint configuration
    print("Test 2: Moving to specific joint angles...")
    test_joints = [30, -45, 60, -30, 90, 0]  # Degrees
    MoveAbsJ(test_joints, speed=0.3, wait_time=3.0)
    
    # Test 3: Linear movement to a Cartesian position
    print("Test 3: Linear movement to Cartesian position...")
    target_xyz = [0.4, 0.2, 0.3]  # X, Y, Z in meters
    target_ypr = [0, 90, 0]       # Yaw, Pitch, Roll in degrees
    MoveLinear(target_xyz, target_ypr, speed=0.2, wait_time=3.0)
    
    # Test 4: Joint movement to another Cartesian position
    print("Test 4: Joint movement to another position...")
    target_xyz = [0.3, -0.2, 0.4]
    target_ypr = [45, 90, 0]
    MoveJoint(target_xyz, target_ypr, speed=0.3, wait_time=3.0)
    
    # Test 5: Test relative linear movement
    print("Test 5: Testing relative linear movement...")
    relative_xyz = [0.05, 0.0, 0.1]  # Move 5cm in X, 10cm up in Z
    MoveRelLinear(relative_xyz, speed=0.1, wait_time=2.0)
    
    # Test 6: Test single joint movement
    print("Test 6: Testing single joint movement...")
    MoveSingleJ(joint_number=1, relative_angle=15, speed=0.2, wait_time=2.0)  # Move joint 1 by 15 degrees
    
    # Test 7: Test gripper opening and closing
    print("Test 7: Testing gripper...")
    print("Opening gripper...")
    GripperSet(100, wait_time=2.0)  # 100% open
    
    print("Partially closing gripper...")
    GripperSet(50, wait_time=2.0)   # 50% closed
    
    print("Closing gripper more...")
    GripperSet(20, wait_time=2.0)   # 20% closed (80% grip)
    
    print("Opening gripper again...")
    GripperSet(100, wait_time=2.0)  # 100% open
    
    # Test 8: Return to home
    print("Test 8: Returning to home position...")
    home_joints = [0, 0, 0, 0, 0, 0]
    MoveAbsJ(home_joints, speed=0.4, wait_time=2.0)
    
    print("=== All tests completed successfully! ===")

def main():
    try:
        print("Robot Test Script Starting...")
        print("Make sure the robot simulation is running!")
        print()
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Run the movement tests
        test_robot_movements()
        
        print("Test script finished successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during execution: {e}")
        print("Make sure the robot simulation and ROS2 environment are properly set up")

if __name__ == "__main__":
    main()