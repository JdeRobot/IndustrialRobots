#!/usr/bin/env python3

"""
Simple test script for the JdeRobot Pick and Place exercise
Uses the HAL API to test basic robot movements
"""

import time
import sys
from HAL import *
from PERCEPTION import *

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
    # ===== Easy knobs =====
    COLORS = ["red", "green", "blue", "purple"]
    SHAPES = ["sphere", "cylinder"]

    # Per-color presets (RGB ranges). Tune here once.
    COLOR_PRESETS = {
        "red":    dict(rmin=100, rmax=255, gmin=0,   gmax=20,  bmin=0,   bmax=20),
        "green":  dict(rmin=0,   rmax=20,  gmin=100, gmax=255, bmin=0,   bmax=20),
        "blue":   dict(rmin=0,   rmax=20,  gmin=0,   gmax=20,  bmin=100, bmax=255),
        "purple": dict(rmin=100, rmax=255, gmin=0,   gmax=100, bmin=10, bmax=255)
    }

    # ===== Your test =====
    print("=== Test 3 color & shape filter ===\n")
    perception = PerceptionNode()
    perception.load_models_info()

    reset(perception=perception)

    print(perception.object_list)

    # COLORS = ["red", "green", "blue", "purple"]
    # SHAPES = ["sphere", "cylinder"]
    # Pick by index:
    color_idx = 1   # COLORS[3] -> "purple"
    shape_idx = 1   # SHAPES[1] -> "cylinder"
    color_name = COLORS[color_idx]
    shape_name = SHAPES[shape_idx]

    object_name = f"{color_name}_{shape_name}"
    print(object_name)

    color_param = COLOR_PRESETS[color_name].copy()

    radius, width, length, shape, color = perception.get_object_info(object_name)
    print(radius, width, length, shape, color)

    # Start filters using presets; override any single value ad hoc if needed:
    # e.g., _send_color_filter(perception, color_name, override={"gmin": 80})
    perception.start_color_filter(
        color=color_name,
        rmax=color_param["rmax"], rmin=color_param["rmin"],
        gmax=color_param["gmax"], gmin=color_param["gmin"],
        bmax=color_param["bmax"], bmin=color_param["bmin"],
    )
    time.sleep(2)

    perception.start_shape_filter(color=color,shape=shape,radius=0.01)
    time.sleep(2)

    print("\n=== Test 3 Finish ===\n")
    print("Test script finished successfully!")


if __name__ == "__main__":
    main()