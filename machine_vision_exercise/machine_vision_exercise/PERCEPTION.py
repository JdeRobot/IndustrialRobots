#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import os
import yaml
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs.tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from pcl_filter_msgs.msg import ColorFilter, ShapeFilter

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import copy
import math

from HAL import *


class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color


class WorkSpace:
    def __init__(self, x, y, z, min_r, max_r, min_z):
        self.x = x
        self.y = y
        self.z = z
        self.min_r = min_r
        self.max_r = max_r
        self.min_z = min_z


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.object_list = {}
        self.goal_list = {}
        self.color_shape_converter = {}
        
        # Initialize color and shape converters
        self.color_shape_converter["red"] = 1
        self.color_shape_converter["green"] = 2
        self.color_shape_converter["blue"] = 3
        self.color_shape_converter["purple"] = 4
        self.color_shape_converter["sphere"] = 1
        self.color_shape_converter["cylinder"] = 2
        
        # Publishers
        self.message_pub = self.create_publisher(String, '/gui_message', 10)
        self.updatepose_pub = self.create_publisher(Bool, '/updatepose', 10)
        self.color_filter_pub = self.create_publisher(ColorFilter, '/start_color_filter', 10)
        self.shape_filter_pub = self.create_publisher(ShapeFilter, '/start_shape_filter', 10)
        self.env_scan_pub = self.create_publisher(Bool, '/trigger_env_scan', 10)
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        # Important: let TransformListener run a background spinner
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        
        # Load configuration
        self.load_models_info()
        self.set_target_info()
        self.get_workspace()
        
        self.get_logger().info('Perception node initialized')

    def load_models_info(self):
        """Load object information from YAML configuration file"""
        try:
            # In ROS2, we need to use get_package_share_directory
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'models_info.yaml')
            
            with open(filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]
                robot_roll = objects_info["robot"]["pose"]["roll"]
                robot_pitch = objects_info["robot"]["pose"]["pitch"]
                robot_yaw = objects_info["robot"]["pose"]["yaw"]

                objects = objects_info["objects"]
                objects_name = objects.keys()
                
                for object_name in objects_name:
                    name = object_name
                    shape = objects[name]["shape"]
                    color = objects[name]["color"]

                    x = objects[name]["pose"]["x"]
                    y = objects[name]["pose"]["y"]
                    z = objects[name]["pose"]["z"]
                    roll = objects[name]["pose"]["roll"]
                    pitch = objects[name]["pose"]["pitch"]
                    yaw = objects[name]["pose"]["yaw"]
                    object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)

                    p = PoseStamped()
                    p.header.frame_id = "base_link"  # Default planning frame
                    p.header.stamp = self.get_clock().now().to_msg()

                    p.pose.position.x = float(x - robot_x)
                    p.pose.position.y = float(y - robot_y)
                    p.pose.position.z = float(z - robot_z)

                    q = quaternion_from_euler(roll, pitch, yaw)
                    p.pose.orientation = Quaternion()
                    p.pose.orientation.x = float(q[0])
                    p.pose.orientation.y = float(q[1])
                    p.pose.orientation.z = float(q[2])
                    p.pose.orientation.w = float(q[3])

                    if shape == "box":
                        x_size = objects[name]["size"]["x"]
                        y_size = objects[name]["size"]["y"]
                        z_size = objects[name]["size"]["z"]
                        p.pose.position.z += float(z_size/2)

                        height = z_size
                        width = y_size
                        length = x_size
                        self.object_list[name] = Object(p.pose, object_pose, height, width, length, shape, color)

                    elif shape == "cylinder":
                        height = objects[name]["size"]["height"]
                        radius = objects[name]["size"]["radius"]
                        p.pose.position.z += float(height/2)
                        self.object_list[name] = Object(p.pose, object_pose, height, radius*2, radius*2, shape, color)

                    elif shape == "sphere":
                        radius = objects[name]["size"]
                        p.pose.position.z += float(radius)
                        self.object_list[name] = Object(p.pose, object_pose, radius*2, radius*2, radius*2, shape, color)
                        
        except Exception as e:
            self.get_logger().error(f'Failed to load models info: {e}')

    def set_target_info(self):
        """Load target information from YAML configuration file"""
        try:
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'models_info.yaml')
            
            with open(filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]

                targets = objects_info["targets"]
                target_names = targets.keys()
                
                for name in target_names:
                    position = Point()
                    position.x = float(targets[name]["x"] - robot_x)
                    position.y = float(targets[name]["y"] - robot_y)
                    position.z = float(targets[name]["z"])
                    self.goal_list[name] = position
                    
        except Exception as e:
            self.get_logger().error(f'Failed to load target info: {e}')

    def get_workspace(self):
        """Load workspace configuration"""
        try:
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'joints_setup.yaml')
            
            with open(filename, 'r') as file:
                joints_setup = yaml.safe_load(file)
                workspace = joints_setup["workspace"]

                x = workspace["center"]["x"]
                y = workspace["center"]["y"]
                z = workspace["center"]["z"]
                min_r = workspace["r"]["min"]
                max_r = workspace["r"]["max"]
                min_z = workspace["min_z"]
                self.workspace = WorkSpace(x, y, z, min_r, max_r, min_z)
                
        except Exception as e:
            self.get_logger().error(f'Failed to load workspace info: {e}')

    def send_message(self, message):
        """Send message to GUI"""
        msg = String()
        msg.data = message
        self.message_pub.publish(msg)

    def updatepose_trigger(self, value):
        """Trigger pose update"""
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    def start_color_filter(self, color, rmax, rmin, gmax, gmin, bmax, bmin):
        """Start color filtering"""
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.rmax = rmax
        color_filter.rmin = rmin
        color_filter.gmax = gmax
        color_filter.gmin = gmin
        color_filter.bmax = bmax
        color_filter.bmin = bmin
        color_filter.status = True
        self.color_filter_pub.publish(color_filter)

    def stop_color_filter(self, color):
        """Stop color filtering"""
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.status = False
        self.color_filter_pub.publish(color_filter)

    def start_shape_filter(self, color, shape, radius):
        """Start shape filtering"""
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.radius = radius
        shape_filter.status = True
        self.shape_filter_pub.publish(shape_filter)

    def stop_shape_filter(self, color, shape):
        """Stop shape filtering"""
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.status = False
        self.shape_filter_pub.publish(shape_filter)

    def get_object_list(self):
        """Get list of object names"""
        return list(self.object_list.keys())

    def get_target_list(self):
        """Get list of target names"""
        return list(self.goal_list.keys())

    def get_object_pose(self, object_name):
        """Get object pose"""
        if object_name in self.object_list:
            return copy.deepcopy(self.object_list[object_name].relative_pose)
        return None

    def get_object_info(self, object_name):
        """Get object information"""
        if object_name in self.object_list:
            this_object = copy.deepcopy(self.object_list[object_name])
            height = this_object.height
            width = this_object.width
            length = this_object.length
            shape = this_object.shape
            color = this_object.color
            return height, width, length, shape, color
        return None, None, None, None, None

    def get_target_position(self, target_name):
        """Get target position"""
        if target_name in self.goal_list:
            return self.goal_list[target_name]
        return None

    def gripper_setting_percentage(self, diameter, max_open_m=0.085):
        """
        Convert object radius (in meters) to gripper closure percentage.
        - radius_m: object radius in meters
        - max_open_m: max jaw opening in meters (default 0.085 m for 2F-85)

        Returns: closure percentage [0-100]
        """
        percentage = (1 - (diameter / max_open_m) )* 100.0
        return percentage  # clamp
    
    # def get_object_position(self, object_name):
    #     """Get object position using TF2"""
    #     frame_id = object_name
    #     try:
    #         # Use TF2 to get transform
    #         transform = self.tf_buffer.lookup_transform(
    #             'world', frame_id, rclpy.time.Time(), 
    #             timeout=rclpy.duration.Duration(seconds=1.0)
    #         )
            
    #         position = Point()
    #         position.x = float(transform.transform.translation.x)
    #         position.y = float(transform.transform.translation.y)
    #         position.z = float(transform.transform.translation.z)
            
    #         self.get_logger().info("*************************************")
    #         self.get_logger().info(f"Detected position of {object_name}:")
    #         self.get_logger().info(f"[{position.x}, {position.y}, {position.z}]")
    #         self.get_logger().info("*************************************")
            
    #         return position
            
    #     except Exception as e:
    #         self.get_logger().error(f"Cannot find desired object {object_name}: {e}")
    #         return None

    def get_object_position(self, object_name):
        """Get object position using TF2"""
        frame_id = object_name
        try:
            # Use TF2 to get transform
            transform = self.tf_buffer.lookup_transform(
                'world', frame_id, rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            x = float(transform.transform.translation.x)
            y = float(transform.transform.translation.y)
            z = float(transform.transform.translation.z + 0.03)
            
            position = [x, y, z]
            
            self.get_logger().info("*************************************")
            self.get_logger().info(f"Detected position of {object_name}:")
            self.get_logger().info(f"{position}")
            self.get_logger().info("*************************************")
            
            return position
            
        except Exception as e:
            self.get_logger().error(f"Cannot find desired object {object_name}: {e}")
            return None

    def is_inside_workspace(self, x, y, z):
        """Check if position is inside workspace"""
        if z > self.workspace.min_z:
            dx = x - self.workspace.x
            dy = y - self.workspace.y
            dz = z - self.workspace.z
            r = math.sqrt(dx**2 + dy**2 + dz**2)
            if self.workspace.min_r < r < self.workspace.max_r:
                return True
        return False

    def pose2msg(self, x, y, z, roll, pitch, yaw):
        """Convert pose parameters to geometry_msgs/Pose"""
        pose = Pose()
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        return pose

    def msg2pose(self, pose):
        """Convert geometry_msgs/Pose to pose parameters"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw, x, y, z

    def pose2msg_deg(self, x, y, z, roll, pitch, yaw):
        """Convert pose parameters (degrees) to geometry_msgs/Pose"""
        pose = Pose()
        quat = quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        return pose

    def msg2pose_deg(self, pose):
        """Convert geometry_msgs/Pose to pose parameters (degrees)"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = np.rad2deg(euler[0])
        pitch = np.rad2deg(euler[1])
        yaw = np.rad2deg(euler[2])
        return roll, pitch, yaw, x, y, z
    
    def start_environment_scan(self):
        """Start environment scanning for collision detection"""
        msg = Bool()
        msg.data = True
        self.env_scan_pub.publish(msg)
        self.get_logger().info("Environment scanning started")
        self.send_message("Environment scanning started")

    def stop_environment_scan(self):
        """Stop environment scanning"""
        msg = Bool()
        msg.data = False
        self.env_scan_pub.publish(msg)
        self.get_logger().info("Environment scanning stopped")
        self.send_message("Environment scanning stopped")

    def buildmap(self):
        """
        Build a map of the workspace by moving robot to scanning positions
        This function moves the robot through key positions to get different viewpoints
        for comprehensive workspace scanning and object detection
        """
        print("Starting workspace mapping procedure...")
        
        # Step 1: Go to home position
        back_to_home()

        # Start environment scanning
        self.start_environment_scan()
        time.sleep(1.0)

        # Step 2: Move to scanning position
        # This position provides a good overview of the workspace
        print("Moving to scanning position...")
        MoveAbsJ([180.00, -90.0, 0.0, 0.0, -60.0, 0.0], 0.1, 1)

        # Step 3: Wait for sensors to stabilize and capture data
        time.sleep(2)
        self.stop_environment_scan()
        # Step 4: Trigger comprehensive workspace scan
        # detected_objects = scan_workspace()
        
        # Step 5: Return to home position
        back_to_home()
        
        print("Workspace mapping completed")
        
        # return detected_objects

def main(args=None):
    rclpy.init(args=args)
    
    perception_node = PerceptionNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(perception_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()