#!/usr/bin/env python3

"""
Simple test script for the JdeRobot Pick and Place exercise
Uses the HAL API to test basic robot movements
"""

import time
import sys
from HAL import *

def get_object_position(self, object_name):
    # Step 1: Get object info
    height, width, length, shape, color = get_object_info(object_name)

    # Step 2: Start color filter (tuned RGB ranges—adjust as needed)
    self.pick_place.start_color_filter(color, rmax=120, rmin=0, gmax=255, gmin=100, bmax=120, bmin=0)
    time.sleep(1.0)

    # Step 3: Start shape filter
    radius = max(width, length) / 2.0
    start_shape_filter(color, shape, radius)
    time.sleep(1.0)

    # Step 4: Get position
    object_frame = "{}_{}".format(color, shape)
    position = get_object_position(object_frame)
    while not position:
        time.sleep(0.5)
        position = get_object_position(object_frame)

    # Step 5: Stop filters
    stop_shape_filter(color, shape)
    stop_color_filter(color)

    return position

def get_object_position(self, object_name):
        
        # Step 1: Get object info
        height, width, length, shape, color = Pick_Place.get_object_info(object_name)

        # Step 2: Start color filter (tuned RGB ranges—adjust as needed)
        Pick_Place.start_color_filter(color, rmax=120, rmin=0, gmax=255, gmin=100, bmax=120, bmin=0)
        time.sleep(1.0)

        # Step 3: Start shape filter
        radius = max(width, length) / 2.0
        Pick_Place.start_shape_filter(color, shape, radius)
        time.sleep(1.0)

        # Step 4: Get position
        # object_frame = "{}_{}".format(color, shape)
        position = Pick_Place.get_object_position(object_name)
        while not position:
            time.sleep(0.5)
            position = Pick_Place.get_object_position(object_name)

        # Step 5: Stop filters
        Pick_Place.stop_shape_filter(color, shape)
        Pick_Place.stop_color_filter(color)

        return position

def main():

##########################################################################################

    # print("Robot Test Script Starting...")
    # print("Make sure the robot simulation is running!")
    # print()

    # print("=== Test 1 Build map ===")
    # print()

    # buildmap()

    # print("=== Test 1 Finish ===")
    # print()

##########################################################################################


    # print("=== Test 2 get model info ===")
    # print()

    # object_list, goal_list = load_objects()
    # print(object_list)
    # print(goal_list)

    # height, width, length, shape, color = get_object_info("green_cylinder")
    # print(height)
    # print(width)
    # print(length)
    # print(shape)
    # print(color)

    # print("=== Test 2 Finish ===")
    # print()
    
    # print("Test script finished successfully!")


##########################################################################################


    print("=== Test 3 color & shape filter ===")
    print()



    print("=== Test 3 Finish ===")
    print()
    
    print("Test script finished successfully!")
        

if __name__ == "__main__":
    main()