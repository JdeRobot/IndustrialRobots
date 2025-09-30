#!/usr/bin/env python3

"""
Minimal Pick and Place Algorithm using HAL API
Skips gripper operations and scan_workspace - focuses on movement and link attacher only
"""

import time
import sys
from PERCEPTION import *
from HAL import *

# Import the HAL API functions
try:
    from HAL import (MoveAbsJ, MoveLinear, MoveJoint, MoveRelLinear, 
                     MoveSingleJ, get_TCP_pose, get_Joint_states, attach, detach, 
                     back_to_home, set_home_position)
    print("HAL API imported successfully!")
except ImportError as e:
    print(f"Error importing HAL: {e}")
    sys.exit(1)

def minimal_pick_and_place():
    """Execute minimal pick and place operation - movement and attach only"""
    
    print("=== MINIMAL PICK AND PLACE OPERATION ===")
    print("Skipping gripper operations and workspace scanning")
    print("Using only movements and link attacher")
    
    # Object and target positions
    object_pos = [0.65, 0.09, 1.01]    # Red cylinder position
    target_pos = [-0.44, -0.06, 1.0]   # Target position
    
    # Orientations
    down_orientation = [0, 90, 0]      # Gripper pointing down
    
    try:
        # Step 1: Set up and go to home position
        print("\n1. Setting up home position and moving there...")
        set_home_position([0.0, -90.0, 0.0, 0.0, -90.0, 0.0])
        # Use MoveAbsJ instead of back_to_home to avoid gripper operations
        MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.5, 2.0)
        
        # Step 2: Skip workspace scanning
        print("\n2. Skipping workspace scanning...")
        print("Moving directly to pick sequence")
        
        # Step 3: Move to pre-pick position (approach from side)
        print("\n3. Moving to pre-pick position...")
        pre_pick = [object_pos[0] - 0.1, object_pos[1], object_pos[2] + 0.2]
        MoveJoint(pre_pick, down_orientation, 0.3, 2.0)
        
        # Step 4: Move to position above object (safe pick position)
        print("\n4. Moving to safe position above object...")
        above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
        MoveJoint(above_object, down_orientation, 0.3, 2.0)
        
        # Step 5: Linear approach to object
        print("\n5. Linear approach to object...")
        approach_object = [object_pos[0], object_pos[1], object_pos[2] + 0.05]
        MoveLinear(approach_object, down_orientation, 0.1, 1.5)
        
        # Step 6: Move to object center (final approach)
        print("\n6. Final approach to object center...")
        at_object = [object_pos[0], object_pos[1], object_pos[2] - 0.02]
        MoveLinear(at_object, down_orientation, 0.05, 1.0)
        
        # Step 7: Attach object (skip gripper, use link attacher only)
        print("\n7. Attaching object using link attacher...")
        try:
            attach('red_cylinder')
            print("✓ Object attached successfully!")
        except Exception as e:
            print(f"✗ Attach failed: {e}")
            print("Continuing anyway...")
        
        # Step 8: Lift object to safe height
        print("\n8. Lifting object to safe height...")
        MoveRelLinear([0, 0, 0.15], 0.1, 2.0)
        
        # Step 9: Move to pre-place position (approach target area)
        print("\n9. Moving to pre-place position...")
        pre_place = [target_pos[0] - 0.1, target_pos[1], target_pos[2] + 0.2]
        MoveJoint(pre_place, down_orientation, 0.3, 2.5)
        
        # Step 10: Move to safe position above target
        print("\n10. Moving to safe position above target...")
        above_target = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
        MoveJoint(above_target, down_orientation, 0.3, 2.0)
        
        # Step 11: Linear approach to target
        print("\n11. Linear approach to target...")
        approach_target = [target_pos[0], target_pos[1], target_pos[2] + 0.05]
        MoveLinear(approach_target, down_orientation, 0.1, 1.5)
        
        # Step 12: Place object at target
        print("\n12. Placing object at target...")
        at_target = [target_pos[0], target_pos[1], target_pos[2] + 0.01]
        MoveLinear(at_target, down_orientation, 0.05, 1.0)
        
        # Step 13: Release object (open gripper will auto-detach)
        print("\n13. Object placed! (Auto-detach when gripper opens)")
        print("Note: Object should detach automatically when gripper opens to 100%")
        
        # Step 14: Move up from placed object
        print("\n14. Moving up from placed object...")
        MoveRelLinear([0, 0, 0.1], 0.1, 2.0)
        
        # Step 15: Return to home position
        print("\n15. Returning to home position...")
        MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.4, 2.0)
        
        print("\n=== ✓ PICK AND PLACE COMPLETED SUCCESSFULLY! ===")
        print("Object should now be at the target position")
        
    except Exception as e:
        print(f"\n✗ Error occurred: {e}")
        print("Attempting emergency return to home...")
        try:
            MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.5, 1.0)
            print("Emergency return completed")
        except:
            print("Emergency return failed")

def test_movement_sequence():
    """Test just the movement sequence without pick/place"""
    
    print("=== TESTING MOVEMENT SEQUENCE ===")
    
    try:
        # Home position
        print("\n1. Going to home...")
        set_home_position([0.0, -90.0, 0.0, 0.0, -90.0, 0.0])
        MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.5, 2.0)
        
        # Test position near object
        print("\n2. Moving near object...")
        near_object = [0.5, 0.1, 1.2]
        down_orient = [0, 90, 0]
        MoveJoint(near_object, down_orient, 0.3, 2.0)
        
        # Test linear movement
        print("\n3. Testing linear movement...")
        MoveRelLinear([0.05, 0.0, -0.1], 0.1, 1.5)
        
        # Move to target area
        print("\n4. Moving to target area...")
        near_target = [-0.3, 0.0, 1.2]
        MoveJoint(near_target, down_orient, 0.3, 2.0)
        
        # Return home
        print("\n5. Returning home...")
        MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.4, 2.0)
        
        print("\n=== ✓ MOVEMENT SEQUENCE TEST COMPLETED ===")
        
    except Exception as e:
        print(f"\n✗ Movement test error: {e}")

def test_attach_only():
    """Test only the attach function"""
    
    print("=== TESTING ATTACH FUNCTION ONLY ===")
    
    try:
        print("Testing attach function...")
        attach('red_cylinder')
        print("✓ Attach function called successfully")
        
        print("Waiting 3 seconds...")
        time.sleep(3.0)
        
        print("Test completed - check if object is attached to robot")
        
    except Exception as e:
        print(f"✗ Attach test failed: {e}")

def get_robot_status():
    """Get current robot status"""
    
    print("=== ROBOT STATUS ===")
    
    try:
        # Get TCP pose
        xyz, ypr = get_TCP_pose()
        if xyz and ypr:
            print(f"TCP Position (XYZ): {xyz}")
            print(f"TCP Orientation (YPR): {ypr}")
        
        # Get joint states
        joints = get_Joint_states()
        if joints:
            print(f"Joint States (degrees): {joints}")
            
    except Exception as e:
        print(f"Status error: {e}")

def reset(perception):
        # Stop all color filters first
    perception.stop_color_filter(color="red")
    time.sleep(0.5)  # Wait for stop to process

    perception.stop_color_filter(color="green")
    time.sleep(0.5)

    perception.stop_color_filter(color="blue") 
    time.sleep(0.5)

    perception.stop_color_filter(color="purple")
    time.sleep(0.5)

    perception.stop_shape_filter(color="red",shape="sphere")
    time.sleep(0.5)
    perception.stop_shape_filter(color="red",shape="cylinder")
    time.sleep(0.5)

    perception.stop_shape_filter(color="green",shape="sphere")
    time.sleep(0.5)
    perception.stop_shape_filter(color="green",shape="cylinder")
    time.sleep(0.5)

    perception.stop_shape_filter(color="blue",shape="sphere")
    time.sleep(0.5)
    perception.stop_shape_filter(color="blue",shape="cylinder")
    time.sleep(0.5)

    perception.stop_shape_filter(color="purple",shape="sphere")
    time.sleep(0.5)
    perception.stop_shape_filter(color="purple",shape="cylinder")
    time.sleep(0.5)


def main():

    COLORS = ["red", "green", "blue", "purple"]
    SHAPES = ["sphere", "cylinder"]
    TARGETS = ["target1","target2","target3","target4","target5","target6","target7","target8","target9","target10","target11","target12","target13","target14","target15","target16"]

    # Per-color presets (RGB ranges). Tune here once.
    COLOR_PRESETS = {
        "red":    dict(rmin=100, rmax=255, gmin=0,   gmax=20,  bmin=0,   bmax=20),
        "green":  dict(rmin=0,   rmax=20,  gmin=100, gmax=255, bmin=0,   bmax=20),
        "blue":   dict(rmin=0,   rmax=20,  gmin=0,   gmax=20,  bmin=100, bmax=255),
        "purple": dict(rmin=100, rmax=255, gmin=0,   gmax=100, bmin=10, bmax=255)
    }

    # ===== Your test =====
    perception = PerceptionNode()
    perception.load_models_info()
    reset(perception=perception)

    # COLORS = ["red", "green", "blue", "purple"]
    # SHAPES = ["sphere", "cylinder"]
    # Pick by index:
    color_idx = 1   # COLORS[3] -> "purple"
    shape_idx = 1   # SHAPES[1] -> "cylinder"
    target_idx = 13

    color_name = COLORS[color_idx]
    shape_name = SHAPES[shape_idx]
    target_name = TARGETS[target_idx - 1]

    object_name = f"{color_name}_{shape_name}"
    print(object_name)

    color_param = COLOR_PRESETS[color_name].copy()

    length, width, diameter, shape, color = perception.get_object_info(object_name)
    print(diameter, width, length, shape, color)

    # object_pos = [0.65, 0.09, 1.01]    # Red cylinder position
    # target_pos = [-0.44, -0.06, 1.0]   # Target position

    object_pos = []
    print(perception.goal_list)
    target_point = perception.get_target_position(target_name)
    target_pos = [target_point.x, target_point.y, target_point.z]
    print(target_pos)

    # Orientations
    down_orientation = [0, 90, 0]      # Gripper pointing down


    # Step 1: Set up and go to home position
    print("\n1. Setting up home position and moving there...")
    set_home_position([0.0, -90.0, 0.0, 0.0, -90.0, 0.0])
    # Use MoveAbsJ instead of back_to_home to avoid gripper operations
    # MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.5, 2.0)
    back_to_home()
    
    # # Step 2: Skip workspace scanning
    # perception.buildmap()
    time.sleep(1.0)
    # GripperSet(100, 1.0)
    
#     # Step 3: Move to pre-pick position (approach from side)
    print("\n3. Moving to pre-pick position...")
    MoveAbsJ([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 1.0)



#################################################################################################################
    # # Step 4: Move to position above object (safe pick position)



    # Start filters using presets; override any single value ad hoc if needed:
    # e.g., _send_color_filter(perception, color_name, override={"gmin": 80})
    perception.start_color_filter(
        color=color_name,
        rmax=color_param["rmax"], rmin=color_param["rmin"],
        gmax=color_param["gmax"], gmin=color_param["gmin"],
        bmax=color_param["bmax"], bmin=color_param["bmin"],
    )
    time.sleep(1)

    perception.start_shape_filter(color=color,shape=shape,radius=diameter/2)
    time.sleep(1)


    object_pos = perception.get_object_position(object_name)

    print(object_pos)

    reset(perception)
#################################################################################################################

    print("\n4. Moving to safe position above object...")
    above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
    MoveJoint(above_object, down_orientation, 0.3, 1.0)
    
    # Step 5: Linear approach to object
    print("\n5. Linear approach to object...")
    approach_object = [object_pos[0], object_pos[1], object_pos[2]]
    # MoveLinear(approach_object, down_orientation, 0.3, 1)
    MoveJoint(approach_object, down_orientation, 0.3, 1.0)



    
    # Step 7: Attach object (skip gripper, use link attacher only)

    print("\n7. Attaching object using link attacher...")
    percentage = perception.gripper_setting_percentage(diameter)
    GripperSet(percentage, 1.0)
    attach(object_name)
    print("✓ Object attached successfully!")
    
    # Step 8: Lift object to safe height

    above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
    MoveJoint(above_object, down_orientation, 0.3, 1.0)
    MoveAbsJ([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 1.0)
    
    # Step 9: Move to pre-place position (approach target area)

    print("\n9. Moving to pre-place position...")
    MoveAbsJ([180.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 1.0)
    above_target = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
    MoveJoint(above_target, down_orientation, 0.5, 1.0)
    
    # Step 10: Move to safe position above target
    print("\n11. Linear approach to target...")
    approach_target = [target_pos[0], target_pos[1], target_pos[2]]
    MoveLinear(approach_target, down_orientation, 0.1, 1.0)
    
    # Step 11: Linear approach to target
    detach()
    GripperSet(0, 1.0)
    
    # Step 12: Place object at target
    print("\n11. Linear approach to target...")
    approach_target = [target_pos[0], target_pos[1], target_pos[2]+ 0.15]
    MoveLinear(approach_target, down_orientation, 0.1, 1.5)
    MoveAbsJ([180.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 1.0)
    
    # Step 13: Release object (open gripper will auto-detach)
    back_to_home()
    


if __name__ == "__main__":
    main()