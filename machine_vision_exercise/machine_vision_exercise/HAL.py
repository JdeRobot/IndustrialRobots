print("HAL initializing", flush=True)
##############################################################################
# JdeROBOT ROBOTICS ACADEMY (http://jderobot.github.io/RoboticsAcademy/)
#  API PICK and PLACE exercise, including:
#   Robot Info: get_TCP_pose, get_Joint_states
#   Kinematics: MoveAbsJ, MoveJoint, MoveLinear, MoveSingleJ 
#               MoveRelLinear, MoveRelReor
#   Gripper: GripperSet, attach, dettach
#   Perception: Color and Shape filtering capabilities
#   
#   VERSION: 1.1
# 	DATE: 	 April 21, 2025
#   AUTHOR:  Diego Martin (diego.martin.martin@gmail.com)
# 
# ======= Acknowledgments =======
#  IFRA-Cranfield nice "ROS2 Sim-to-Real Robot Control" package 
#  URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
##############################################################################

import sys, os, time, math
import rclpy
import numpy as np
import yaml
import copy
from rclpy.time import Time

from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2srrc_data.msg import Robpose
from linkattacher_msgs.srv import AttachLink, DetachLink
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

# Build PATH and import Python classes from IFRA package:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')

PATH_ROB = PATH + "/robot"
sys.path.append(PATH_ROB)
from robot import RBT

PATH_EE = PATH + "/endeffector"
sys.path.append(PATH_EE)
from robotiq_ur import RobotiqGRIPPER

# Import ROS2 Custom Messages from IFRA package:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# TODO: Replace with appropriate ROS2 equivalents for perception
# These would need to be converted to ROS2 message types
from pcl_filter_msgs.msg import ColorFilter, ShapeFilter

# Initialization
rclpy.init(args=None)
UR5 = RBT()

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler 

#################################### ROBOT KINEMATICS ###################################################
# MoveAbsJ. Absolute Joints in degrees, speed max 1.0, wait time after movement in seconds
def MoveAbsJ(absolute_joints, speed, wait_time):
    
    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = float(speed)

    INPUT = Joints()
    INPUT.joint1 = float(absolute_joints[0])
    INPUT.joint2 = float(absolute_joints[1])   
    INPUT.joint3 = float(absolute_joints[2])
    INPUT.joint4 = float(absolute_joints[3])
    INPUT.joint5 = float(absolute_joints[4])
    INPUT.joint6 = float(absolute_joints[5])
    ACTION.movej = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Robot moved to Joint Angular Goal: {absolute_joints}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

# MoveLinear. Linear movement to absolute pose XYZ with YPR absolute orientation in degrees
# Speed max 1.0, wait time after movement in seconds
def MoveLinear(abs_xyz, abs_ypr, speed, wait_time):
    
    roll = math.radians(abs_ypr[0]) # Converts XYR to rad
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])
    
    # Quaternion from YPT in rad
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)    
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) 
    
    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx 
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("LIN", float(speed), InputPose)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Robot moved linearly to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")
    
# MoveJoint. Point-to-point movement to absolute pose XYZ with YPR absolute orientation in degrees
# Speed max 1.0, wait time after movement in seconds
def MoveJoint(abs_xyz, abs_ypr, speed, wait_time):
    
    roll = math.radians(abs_ypr[0]) # Converts XYR to rad
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])
    
    # Quaternion from YPT in rad
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)    
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) 
    
    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx 
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("PTP", float(speed), InputPose)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Robot moved Point-to-Point to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

# MoveRelLinear. Linear movement, relative cartesian coordinates
# Speed max 1.0, wait time after movement in seconds
def MoveRelLinear(relative_xyz, speed, wait_time):
    ACTION = Action()
    ACTION.action = "MoveL"
    ACTION.speed = float(speed)

    INPUT = Xyz()
    INPUT.x = float(relative_xyz[0])
    INPUT.y = float(relative_xyz[1])
    INPUT.z = float(relative_xyz[2])
    ACTION.movel = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Robot moved LINEARLY by a relative increment of : {relative_xyz}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")

# MoveSingleJ. Relative angle in degrees, speed max 1.0, wait time after movement in seconds
def MoveSingleJ(joint_number, relative_angle, speed, wait_time):    
    ACTION = Action()
    ACTION.action = "MoveR"
    ACTION.speed = float(speed)

    INPUT = Joint()
    INPUT.joint = str(joint_number)
    INPUT.value = float(relative_angle)
    ACTION.mover = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Robot moved {joint_number} in {relative_angle} degrees")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")
    
# Relative Reorient given relative Euler Angles
# Speed max 1.0, wait time after movement in seconds
def MoveRelReor(relative_ypr, speed, wait_time):
    ACTION = Action()
    ACTION.action = "MoveROT"
    ACTION.speed = float(speed)

    INPUT = Ypr()
    INPUT.pitch = float(relative_ypr[0])
    INPUT.yaw = float(relative_ypr[1])
    INPUT.roll = float(relative_ypr[2])
    ACTION.moverot = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"TCP reoriented by a relative increment of : {relative_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

###################################### ROBOT INFO ###################################################
def get_TCP_pose():
    """Get current TCP pose in XYZ coordinates and YPR orientation (degrees)"""
    current_pose = UR5.RobGet_POSE()
    if current_pose:
        xyz = [current_pose.x, current_pose.y, current_pose.z]
        # Convert quaternion to YPR
        qx, qy, qz, qw = current_pose.qx, current_pose.qy, current_pose.qz, current_pose.qw
        
        # Convert quaternion to Euler angles (YPR)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        ypr = [math.degrees(yaw), math.degrees(pitch), math.degrees(roll)]
        
        print(f"Current TCP Pose - XYZ: {xyz}, YPR: {ypr}")
        return xyz, ypr
    else:
        print("Failed to get TCP pose")
        return None, None

def get_Joint_states():
    """Get current joint states in degrees"""
    joint_states = UR5.RobGet_JOINTS()
    if joint_states:
        joints = [
            math.degrees(joint_states.joint1),
            math.degrees(joint_states.joint2),
            math.degrees(joint_states.joint3),
            math.degrees(joint_states.joint4),
            math.degrees(joint_states.joint5),
            math.degrees(joint_states.joint6)
        ]
        print(f"Current Joint States (degrees): {joints}")
        return joints
    else:
        print("Failed to get joint states")
        return None
    
###################################### GRIPPER ###################################################
class LinkAttacherClient(Node):
    def __init__(self):
        super().__init__('link_attacher_client')
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attach service not available, waiting again...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detach service not available, waiting again...')
    
    def send_attach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = AttachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name
        
        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_detach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = DetachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name
        
        future = self.detach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# Attach object to gripper. Must be called explicitely
def attach(item):
    link_attacher_client = LinkAttacherClient()
    
    # Attach service call
    attach_response = link_attacher_client.send_attach_request('ur5', 'EE_robotiq_2f85', item, item)
    link_attacher_client.get_logger().info('Attach Response: %s' % attach_response.success)

# Dettach all objects. It is always called when gripper is set to full open (0%)
def dettach(): 
    link_attacher_client = LinkAttacherClient()
    
    # Detach operation for all possible objects when gripper is set to 0%
    link_attacher_client.send_detach_request('ur5', 'EE_robotiq_2f85', 'red_box', 'red_box')
    link_attacher_client.send_detach_request('ur5', 'EE_robotiq_2f85', 'yellow_box', 'yellow_box')
    link_attacher_client.send_detach_request('ur5', 'EE_robotiq_2f85', 'blue_sphere', 'blue_sphere')
    link_attacher_client.send_detach_request('ur5', 'EE_robotiq_2f85', 'green_cylinder', 'green_cylinder')

# Gripper closing and opeining to a given percentage (100% full open, 0% full closed)
# Speed max 1.0, wait time after movement in seconds    
def GripperSet(relative_closure, wait_time):
    ACTION = Action()
    ACTION.action = "MoveG"
    ACTION.speed = float(1) # Gripper speed not working for Robotiq 85, set to 100%

    ACTION.moveg = float(relative_closure)

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    # Print movement results if movement succeeded
    if EXECUTION['Success'] == True:
        print(f"Gripper set to a percentage of: {relative_closure} %")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s")
        if relative_closure == 0:
            dettach() # Automatic object dettach from gripper when full open (0%)
            
    else: 
        print("Gripper closing FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

#################################### PERCEPTION FUNCTIONS ###################################################

#!/usr/bin/env python3

import os
import yaml
import math
import copy
import numpy
import rospkg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import String, Bool
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

# Custom message types - you'll need to define these in your ROS2 package
# from your_package.msg import ColorFilter, ShapeFilter


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


class Pick_Place(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        
        # Initialize object and goal lists
        self.object_list = {}
        self.goal_list = {}
        # Find the base directory of your package (install/share)
        pkg_path = get_package_share_directory("machine_vision_exercise")

        # Build the path down into src/rqt_vacuum_gripper/interfaces/models_info.yaml
        self.filename = os.path.join(pkg_path, "config", "models_info.yaml")
        
        # Load object information
        self._load_objects_info()
        self._set_target_info()
        
        # Set up publishers
        self.message_pub = self.create_publisher(String, '/gui_message', 10)
        self.updatepose_pub = self.create_publisher(Bool, '/updatepose', 10)
        
        # Uncomment these when you have the custom message types defined
        # self.color_filter_pub = self.create_publisher(ColorFilter, '/start_color_filter', 10)
        # self.shape_filter_pub = self.create_publisher(ShapeFilter, '/start_shape_filter', 10)
        
        # Set up TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Load workspace configuration
        self._get_workspace()
        
        # Color and shape converters
        self.color_shape_converter = {
            "red": 1,
            "green": 2,
            "blue": 3,
            "yellow": 4,
            "sphere": 1,
            "cylinder": 2
        }
        
        self.get_logger().info('Pick_Place node initialized')

    def _load_objects_info(self):
        """Load object information from YAML file"""
        try:            
            with open(self.filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                
                # Get robot pose
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]
                robot_roll = objects_info["robot"]["pose"]["roll"]
                robot_pitch = objects_info["robot"]["pose"]["pitch"]
                robot_yaw = objects_info["robot"]["pose"]["yaw"]
                
                # Process objects
                objects = objects_info["objects"]
                for object_name, obj_data in objects.items():
                    name = object_name
                    shape = obj_data["shape"]
                    color = obj_data["color"]
                    
                    # Object pose
                    x = obj_data["pose"]["x"]
                    y = obj_data["pose"]["y"]
                    z = obj_data["pose"]["z"]
                    roll = obj_data["pose"]["roll"]
                    pitch = obj_data["pose"]["pitch"]
                    yaw = obj_data["pose"]["yaw"]
                    
                    object_pose = self.pose2msg(roll, pitch, yaw, x, y, z)
                    
                    # Create relative pose
                    p = Pose()
                    p.position.x = x - robot_x
                    p.position.y = y - robot_y
                    p.position.z = z - robot_z
                    
                    q = quaternion_from_euler(roll, pitch, yaw)
                    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    
                    # Handle different shapes
                    if shape == "box":
                        size_x = obj_data["size"]["x"]
                        size_y = obj_data["size"]["y"]
                        size_z = obj_data["size"]["z"]
                        p.position.z += size_z / 2
                        
                        height = size_z
                        width = size_y
                        length = size_x
                        self.object_list[name] = Object(p, object_pose, height, width, length, shape, color)
                        
                    elif shape == "cylinder":
                        height = obj_data["size"]["height"]
                        radius = obj_data["size"]["radius"]
                        p.position.z += height / 2
                        self.object_list[name] = Object(p, object_pose, height, radius*2, radius*2, shape, color)
                        
                    elif shape == "sphere":
                        radius = obj_data["size"]
                        p.position.z += radius
                        self.object_list[name] = Object(p, object_pose, radius*2, radius*2, radius*2, shape, color)
                        
        except FileNotFoundError:
            self.get_logger().error(f'Could not find models_info.yaml file at {filename}')
        except Exception as e:
            self.get_logger().error(f'Error loading objects info: {str(e)}')

    def _set_target_info(self):
        """Load target information from YAML file"""
        try:            
            with open(self.filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]
                
                targets = objects_info["targets"]
                for name, target_data in targets.items():
                    position = Point()
                    position.x = target_data["x"] - robot_x
                    position.y = target_data["y"] - robot_y
                    position.z = target_data["z"] - robot_z
                    self.goal_list[name] = position
                    
        except Exception as e:
            self.get_logger().error(f'Error loading target info: {str(e)}')

    def _get_workspace(self):
        """Load workspace configuration from YAML file"""
        try:            
            with open(self.filename, 'r') as file:
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
            self.get_logger().error(f'Error loading workspace config: {str(e)}')

    # Publisher methods
    def send_message(self, message):
        """Send a message via ROS2 publisher"""
        msg = String()
        msg.data = message
        self.message_pub.publish(msg)
        
    def updatepose_trigger(self, value):
        """Trigger pose update"""
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    # Uncomment these methods when you have the custom message types defined

    def start_color_filter(self, color, rmax, rmin, gmax, gmin, bmax, bmin):
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
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.status = False
        self.color_filter_pub.publish(color_filter)

    def start_shape_filter(self, color, shape, radius):
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.radius = radius
        shape_filter.status = True
        self.shape_filter_pub.publish(shape_filter)

    def stop_shape_filter(self, color, shape):
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.status = False
        self.shape_filter_pub.publish(shape_filter)
    

    # Getter methods
    def get_object_list(self):
        """Get list of object names"""
        return list(self.object_list.keys())

    def get_target_list(self):
        """Get list of target names"""
        return list(self.goal_list.keys())

    def get_object_pose(self, object_name):
        """Get pose of specific object"""
        if object_name in self.object_list:
            return copy.deepcopy(self.object_list[object_name].relative_pose)
        else:
            self.get_logger().warn(f'Object {object_name} not found')
            return None

    def get_object_info(self, object_name):
        """Get complete info of specific object"""
        if object_name in self.object_list:
            this_object = copy.deepcopy(self.object_list[object_name])
            return (this_object.height, this_object.width, this_object.length, 
                   this_object.shape, this_object.color)
        else:
            self.get_logger().warn(f'Object {object_name} not found')
            return None
        
    def get_object_position(self, object_name):
        """Get object position using TF2 transform lookup"""
        frame_id = object_name
        try:
            # Lookup transform from world to object frame
            transform = self.tf_buffer.lookup_transform(
                'world', 
                frame_id, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract translation
            trans = transform.transform.translation
            
            position = Point()
            position.x = trans.x
            position.y = trans.y
            position.z = trans.z
            
            print("*************************************")
            print("Detected position of " + object_name + ":")
            print([trans.x, trans.y, trans.z])
            print("*************************************")
            
            return position
            
        except Exception as e:
            self.get_logger().info("Cannot find desired object: " + str(e))
            return None


    def get_target_position(self, target_name):
        """Get position of specific target"""
        if target_name in self.goal_list:
            return self.goal_list[target_name]
        else:
            self.get_logger().warn(f'Target {target_name} not found')
            return None

    # Utility methods for pose conversion
    def pose2msg(self, roll, pitch, yaw, x, y, z):
        """Convert roll, pitch, yaw, x, y, z to Pose message"""
        pose = Pose()
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        return pose

    def msg2pose(self, pose):
        """Convert Pose message to roll, pitch, yaw, x, y, z"""
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

    def pose2msg_deg(self, roll, pitch, yaw, x, y, z):
        """Convert degrees to Pose message"""
        pose = Pose()
        quat = quaternion_from_euler(numpy.deg2rad(roll), 
                                   numpy.deg2rad(pitch), 
                                   numpy.deg2rad(yaw))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        return pose

    def msg2pose_deg(self, pose):
        """Convert Pose message to degrees"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = numpy.rad2deg(euler[0])
        pitch = numpy.rad2deg(euler[1])
        yaw = numpy.rad2deg(euler[2])
        return roll, pitch, yaw, x, y, z

    # Workspace validation
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


#################################### UTILITY FUNCTIONS ###################################################

def initialize_perception():
    """Initialize the perception system"""
    print("Initializing perception system...")
    _get_perception_manager()
    print("Perception system ready")

def shutdown_perception():
    """Shutdown the perception system cleanly"""
    global _perception_manager
    if _perception_manager is not None:
        _perception_manager.destroy_node()
        _perception_manager = None
    print("Perception system shutdown")


#################################### WORKSPACE MAPPING ###################################################

# Home position configuration (can be modified as needed)
_home_joints = [0.0, -90.0, 0.0, 0.0, -90.0, 0.0]  # Default home position in degrees

def set_home_position(joint_angles):
    """
    Set the home position for the robot
    
    Args:
        joint_angles (list): List of 6 joint angles in degrees [j1, j2, j3, j4, j5, j6]
    """
    global _home_joints
    if len(joint_angles) != 6:
        print("Error: Home position must have exactly 6 joint angles")
        return False
    
    _home_joints = joint_angles.copy()
    print(f"Home position set to: {_home_joints}")
    return True

def get_home_position():
    """
    Get the current home position
    
    Returns:
        list: Current home position joint angles in degrees
    """
    return _home_joints.copy()

def back_to_home():
    """
    Move robot back to the predefined home position
    Uses higher speed (90%) and releases gripper
    """
    print("Moving robot back to home position...")
    perception_mgr = _get_perception_manager()
    perception_mgr.send_message("Returning to home position")
    
    # Move to home position with higher speed
    MoveAbsJ(_home_joints, 0.9, 1.0)
    
    # Release gripper
    GripperSet(100, 0.5)  # 100% open (full release)
    
    print("Robot returned to home position")

def move_joint_arm(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    """
    Move the robot arm using Forward Kinematics (FK) by specifying joint angles
    This is a direct wrapper around MoveAbsJ with default speed and wait time
    
    Args:
        joint_0 to joint_5 (float): Joint angles in degrees for joints 1-6
    """
    joint_angles = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5]
    
    # Use MoveAbsJ with moderate speed and wait time
    MoveAbsJ(joint_angles, 0.5, 1.0)
    
    # Trigger pose update for external systems
    perception_mgr = _get_perception_manager()
    perception_mgr.updatepose_trigger(True)

def buildmap():
    """
    Build a map of the workspace by moving robot to scanning positions
    This function moves the robot through key positions to get different viewpoints
    for comprehensive workspace scanning and object detection
    """
    print("Starting workspace mapping procedure...")
    perception_mgr = _get_perception_manager()
    perception_mgr.send_message("Building workspace map")
    
    # Step 1: Go to home position
    back_to_home()
    
    # Step 2: Move to scanning position
    # This position provides a good overview of the workspace
    print("Moving to scanning position...")
    move_joint_arm(180.00, -90.0, 0.0, 0.0, -60.0, 0.0)  # Joint angles in degrees
    
    # Step 3: Wait for sensors to stabilize and capture data
    time.sleep(1.0)
    
    # Step 4: Trigger comprehensive workspace scan
    detected_objects = scan_workspace()
    
    # Step 5: Return to home position
    back_to_home()
    
    print("Workspace mapping completed")
    perception_mgr.send_message("Workspace map built successfully")
    
    return detected_objects

def custom_scan_sequence(scan_positions):
    """
    Perform workspace scanning using custom joint positions
    
    Args:
        scan_positions (list): List of joint angle lists for scanning positions
                              Each position should be [j1, j2, j3, j4, j5, j6] in degrees
    
    Returns:
        dict: Combined results from all scanning positions
    """
    print(f"Starting custom scan sequence with {len(scan_positions)} positions...")
    perception_mgr = _get_perception_manager()
    perception_mgr.send_message("Starting custom scanning sequence")
    
    all_detected_objects = {}
    
    # Start from home
    back_to_home()
    
    for i, position in enumerate(scan_positions):
        print(f"Moving to scan position {i+1}/{len(scan_positions)}")
        
        # Move to scanning position
        if len(position) == 6:
            move_joint_arm(*position)
        else:
            print(f"Warning: Invalid position {i+1}, skipping...")
            continue
        
        # Wait for stabilization
        time.sleep(1.0)
        
        # Scan from this position
        position_results = scan_workspace()
        
        # Merge results (in real implementation, you'd handle coordinate transformations)
        all_detected_objects.update(position_results)
    
    # Return to home
    back_to_home()
    
    print("Custom scan sequence completed")
    perception_mgr.send_message("Custom scanning sequence completed")
    
    return all_detected_objects

class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color

def load_objects():
    # Find the base directory of your package (install/share)
    pkg_path = get_package_share_directory("machine_vision_exercise")

    # Build the path down into src/rqt_vacuum_gripper/interfaces/models_info.yaml
    filename = os.path.join(pkg_path, "config", "models_info.yaml")

    object_list = {}
    goal_list = {}

    with open(filename, "r") as f:
        objects_info = yaml.safe_load(f)

    robot_x = objects_info["robot"]["pose"]["x"]
    robot_y = objects_info["robot"]["pose"]["y"]
    robot_z = objects_info["robot"]["pose"]["z"]

    objects = objects_info["objects"]

    for name, spec in objects.items():
        shape = spec["shape"]
        color = spec["color"]

        x = spec["pose"]["x"]
        y = spec["pose"]["y"]
        z = spec["pose"]["z"]
        roll = spec["pose"]["roll"]
        pitch = spec["pose"]["pitch"]
        yaw = spec["pose"]["yaw"]

        p = PoseStamped()
        p.header.frame_id = "world"
        p.header.stamp = Time().to_msg()

        p.pose.position.x = x - robot_x
        p.pose.position.y = y - robot_y
        p.pose.position.z = z - robot_z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        if shape == "box":
            sx = spec["size"]["x"]
            sy = spec["size"]["y"]
            sz = spec["size"]["z"]
            p.pose.position.z += sz / 2.0
            object_list[name] = Object(p.pose, None, sz, sy, sx, shape, color)

        elif shape == "cylinder":
            height = spec["size"]["height"]
            radius = spec["size"]["radius"]
            p.pose.position.z += height / 2.0
            object_list[name] = Object(p.pose, None, height, radius * 2.0, radius * 2.0, shape, color)

        elif shape == "sphere":
            radius = spec["size"]
            p.pose.position.z += radius
            object_list[name] = Object(p.pose, None, radius * 2.0, radius * 2.0, radius * 2.0, shape, color)

        else:
            print(f"Unknown shape '{shape}' for object '{name}'")

    return object_list, goal_list

def get_object_info(object_name):
    object_list,goal_list = load_objects()
    this_object = copy.deepcopy(object_list[object_name])
    height = this_object.height
    width = this_object.width
    length = this_object.length
    shape = this_object.shape
    color = this_object.color
    return height, width, length, shape, color

