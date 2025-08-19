#!/usr/bin/env python3

import sys
import os
sys.path.append('/home/shu/dev_ws/src/IndustrialRobots/machine_vision_exercise/machine_vision_exercise')

import time
import rclpy
from PERCEPTION import PerceptionNode

def working_yellow_test():
    """
    Working yellow detection test with proper ROS2 initialization
    """
    print("=== Working Yellow Detection Test ===")
    
    # Initialize ROS2 properly
    try:
        rclpy.init()
        print("ROS2 initialized successfully")
    except Exception as e:
        print(f"ROS2 already initialized or error: {e}")
    
    # Test ranges from most restrictive to most permissive
    test_ranges = [
        {"name": "Original_Bright", "r": (200, 255), "g": (200, 255), "b": (0, 50)},
        {"name": "Medium_Bright", "r": (150, 255), "g": (150, 255), "b": (0, 80)},
        {"name": "Relaxed", "r": (120, 255), "g": (120, 255), "b": (0, 120)},
        {"name": "Permissive", "r": (100, 255), "g": (100, 255), "b": (0, 150)},
        {"name": "Very_Wide", "r": (80, 255), "g": (80, 255), "b": (0, 200)},
        {"name": "Ultra_Wide", "r": (60, 255), "g": (60, 255), "b": (0, 255)},
    ]
    
    try:
        perception = PerceptionNode()
        print("PerceptionNode created successfully")
        
        # Stop all filters first
        print("Stopping all existing filters...")
        for color in ["red", "green", "blue", "yellow"]:
            try:
                perception.stop_color_filter(color=color)
                time.sleep(0.3)
            except Exception as e:
                print(f"Warning: Could not stop {color} filter: {e}")
        
        print("\nStarting yellow detection tests...")
        print("Monitor these commands in separate terminals:")
        print("  ros2 topic echo /yellow_filter --once")
        print("  ros2 run rqt_image_view rqt_image_view /yellow_filtered_image")
        print("\nLook for:")
        print("  - 'width: X' where X > 0 in pointcloud (means detection)")
        print("  - White pixels in the image viewer")
        print("=" * 60)
        
        for i, test_range in enumerate(test_ranges, 1):
            print(f"\n--- Test {i}/{len(test_ranges)}: {test_range['name']} ---")
            
            r_min, r_max = test_range['r']
            g_min, g_max = test_range['g'] 
            b_min, b_max = test_range['b']
            
            print(f"RGB Range: R({r_min}-{r_max}) G({g_min}-{g_max}) B({b_min}-{b_max})")
            
            # Apply the filter
            try:
                perception.start_color_filter(
                    color="yellow",
                    rmax=r_max, rmin=r_min,
                    gmax=g_max, gmin=g_min,
                    bmax=b_max, bmin=b_min
                )
                print("✓ Filter applied successfully")
            except Exception as e:
                print(f"✗ Error applying filter: {e}")
                continue
            
            print("Waiting 3 seconds for filter to process...")
            time.sleep(3)
            
            print("Check your terminals now for detection results!")
            
            # Get user feedback
            while True:
                user_input = input("\nResult? (g=good, p=partial, n=none, q=quit): ").lower()
                if user_input in ['g', 'p', 'n', 'q']:
                    break
                print("Please enter g, p, n, or q")
            
            if user_input == 'q':
                print("Quitting test...")
                break
            elif user_input == 'g':
                print(f"🎉 EXCELLENT! {test_range['name']} works well!")
                print(f"Save these parameters:")
                print(f"  rmax={r_max}, rmin={r_min}")
                print(f"  gmax={g_max}, gmin={g_min}")
                print(f"  bmax={b_max}, bmin={b_min}")
                break
            elif user_input == 'p':
                print(f"👍 {test_range['name']} shows partial detection - continuing...")
            else:
                print(f"👎 {test_range['name']} shows no detection - trying next...")
            
            # Stop current filter before next test
            try:
                perception.stop_color_filter(color="yellow")
                time.sleep(0.5)
            except Exception as e:
                print(f"Warning: Could not stop yellow filter: {e}")
        
        # Final cleanup
        perception.stop_color_filter(color="yellow")
        print("\n✓ Test completed - all filters stopped")
        
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            if 'perception' in locals():
                perception.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass

def quick_test_specific_range():
    """
    Quick test of a specific range that should work for Gazebo yellow
    """
    print("=== Quick Test: Gazebo Yellow Range ===")
    
    try:
        rclpy.init()
    except:
        pass
    
    try:
        perception = PerceptionNode()
        
        # Stop all filters
        for color in ["red", "green", "blue", "yellow"]:
            perception.stop_color_filter(color=color)
            time.sleep(0.3)
        
        print("Testing optimized range for Gazebo yellow materials...")
        print("RGB Range: R(80-255) G(80-255) B(0-180)")
        
        perception.start_color_filter(
            color="yellow",
            rmax=255, rmin=80,
            gmax=255, gmin=80,
            bmax=180, bmin=0
        )
        
        print("✓ Filter applied!")
        print("Check these topics now:")
        print("  ros2 topic echo /yellow_filter --once")
        print("  ros2 run rqt_image_view rqt_image_view /yellow_filtered_image")
        
        input("Press Enter when done checking...")
        
        perception.stop_color_filter(color="yellow")
        perception.destroy_node()
        
    except Exception as e:
        print(f"Error: {e}")
    
    try:
        rclpy.shutdown()
    except:
        pass

if __name__ == "__main__":
    print("Yellow Detection Test Options:")
    print("1. Full systematic test (recommended)")
    print("2. Quick test of likely working range")
    
    choice = input("Choose (1 or 2): ").strip()
    
    if choice == "2":
        quick_test_specific_range()
    else:
        working_yellow_test()