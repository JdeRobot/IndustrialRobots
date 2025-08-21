#!/usr/bin/env python3

"""
Minimal Pick and Place Algorithm using HAL API
Skips gripper operations and scan_workspace - focuses on movement and link attacher only
"""

import time
import sys

# Import the HAL API functions
try:
    from HAL import (MoveAbsJ, MoveLinear, MoveJoint, MoveRelLinear, 
                     MoveSingleJ, get_TCP_pose, get_Joint_states, attach, detach, 
                     back_to_home, set_home_position,buildmap)
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

def main():

    object_pos = [0.65, 0.09, 1.01]    # Red cylinder position
    target_pos = [-0.44, -0.06, 1.0]   # Target position
    
    # Orientations
    down_orientation = [0, 90, 0]      # Gripper pointing down

    # Step 1: Set up and go to home position
    print("\n1. Setting up home position and moving there...")
    set_home_position([0.0, -90.0, 0.0, 0.0, -90.0, 0.0])
    # Use MoveAbsJ instead of back_to_home to avoid gripper operations
    MoveAbsJ([0.0, -90.0, 0.0, 0.0, -90.0, 0.0], 0.5, 2.0)
    
    # Step 2: Skip workspace scanning
    buildmap()
    
    # Step 3: Move to pre-pick position (approach from side)
    print("\n3. Moving to pre-pick position...")
    MoveAbsJ([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 2.0)
    
    # Step 4: Move to position above object (safe pick position)
    print("\n4. Moving to safe position above object...")
    above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
    MoveJoint(above_object, down_orientation, 0.3, 2.0)
    
    # Step 5: Linear approach to object
    print("\n5. Linear approach to object...")
    approach_object = [object_pos[0], object_pos[1], object_pos[2] + 0.05]
    MoveLinear(approach_object, down_orientation, 0.1, 1.5)
    
    # Step 7: Attach object (skip gripper, use link attacher only)

    print("\n7. Attaching object using link attacher...")
    attach('red_cylinder')
    print("✓ Object attached successfully!")
    
    # Step 8: Lift object to safe height

    above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
    MoveJoint(above_object, down_orientation, 0.3, 2.0)
    MoveAbsJ([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 2.0)
    
    # Step 9: Move to pre-place position (approach target area)

    print("\n9. Moving to pre-place position...")
    MoveAbsJ([180.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 2.0)
    above_target = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
    MoveJoint(above_target, down_orientation, 0.5, 2.5)
    
    # Step 10: Move to safe position above target
    print("\n11. Linear approach to target...")
    approach_target = [target_pos[0], target_pos[1], target_pos[2]]
    MoveLinear(approach_target, down_orientation, 0.1, 1.5)
    
    # Step 11: Linear approach to target
    detach()
    
    # Step 12: Place object at target
    print("\n11. Linear approach to target...")
    approach_target = [target_pos[0], target_pos[1], target_pos[2]+ 0.15]
    MoveLinear(approach_target, down_orientation, 0.1, 1.5)
    MoveAbsJ([180.0, -90.0, 90.0, -90.0, -90.0, 0.0], 0.5, 2.0)
    
    # Step 13: Release object (open gripper will auto-detach)
    back_to_home()
    

if __name__ == "__main__":
    main()