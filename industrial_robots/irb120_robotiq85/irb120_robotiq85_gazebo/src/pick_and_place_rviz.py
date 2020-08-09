#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
#from moveit_python import *

# Init stuff
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_irb120_robot', anonymous=True)
robot = moveit_commander.RobotCommander()
#scene = moveit_commander.PlanningSceneInterface()
scene = moveit_commander.PlanningSceneInterface("base_link")
arm_group = moveit_commander.MoveGroupCommander("irb_120")
hand_group = moveit_commander.MoveGroupCommander("robotiq_85")

# Publish trajectory in RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

grasp = Grasp()
place = PlaceLocation()
pick_and_place = PickPlaceInterface("irb_120", "robotiq_85")

# Inverse Kinematics (IK): move TCP to given position and orientation
def move_pose_arm(roll,pitch,yaw,x,y,z):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)

    arm_group.stop() # To guarantee no residual movement
    arm_group.clear_pose_targets()

# Move the Robotiq gripper by master axis
def move_joint_hand(gripper_finger1_joint):
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[2] = gripper_finger1_joint # Gripper master axis

    hand_group.go(joint_goal, wait=True)
    hand_group.stop() # To guarantee no residual movement

def openGripper():
    grasp.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
    traj = JointTrajectoryPoint()
    traj.positions.append(0)
    traj.time_from_start = rospy.Duration.from_sec(0.5)
    grasp.pre_grasp_posture.points.append(traj)

def closedGripper():
    grasp.grasp_posture.joint_names.append("gripper_finger1_joint")
    traj = JointTrajectoryPoint()
    traj.positions.append(0.5)
    traj.time_from_start = rospy.Duration.from_sec(0.5)
    grasp.grasp_posture.points.append(traj)

def setting_grasp():
    # Setting grasp pose
    grasp.grasp_pose.header.frame_id = "base_link"
    roll,pitch,yaw,x,y,z = 0, pi/2, 0, 0.5, 0, 0.17
    quat = quaternion_from_euler(roll, pitch, yaw)
    grasp.grasp_pose.pose.orientation.x = quat[0]
    grasp.grasp_pose.pose.orientation.y = quat[1]
    grasp.grasp_pose.pose.orientation.z = quat[2]
    grasp.grasp_pose.pose.orientation.w = quat[3]
    grasp.grasp_pose.pose.position.x = x
    grasp.grasp_pose.pose.position.y = y
    grasp.grasp_pose.pose.position.z = z

    # Setting pre-grasp approach
    grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
    grasp.pre_grasp_approach.direction.vector.z = -0.2
    grasp.pre_grasp_approach.min_distance = 0.01
    grasp.pre_grasp_approach.desired_distance = 0.05

    # Setting post-grasp retreat
    grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
    grasp.post_grasp_retreat.direction.vector.z = 0.2
    grasp.post_grasp_retreat.min_distance = 0.01
    grasp.post_grasp_retreat.desired_distance = 0.05

    grasp.max_contact_force = 100

    openGripper()
    closedGripper()

def setting_place():
    #Setting grasp pose
    # place.grasp_pose.header.frame_id = "base_link"
    # roll,pitch,yaw,x,y,z = 0, pi/2, 0, 0.4, 0, 0.1
    # quat = quaternion_from_euler(roll, pitch, yaw)
    # place.grasp_pose.pose.orientation.x = quat[0]
    # place.grasp_pose.pose.orientation.y = quat[1]
    # place.grasp_pose.pose.orientation.z = quat[2]
    # place.grasp_pose.pose.orientation.w = quat[3]
    # place.grasp_pose.pose.position.x = x
    # place.grasp_pose.pose.position.y = y
    # place.grasp_pose.pose.position.z = z

    # # Setting pre-place approach
    # place.pre_grasp_approach.direction.header.frame_id = "base_link"
    # place.pre_grasp_approach.direction.vector.z = -1.0
    # place.pre_grasp_approach.min_distance = 0.01
    # place.pre_grasp_approach.desired_distance = 0.05

    # # Setting post-place retreat
    # place.post_grasp_retreat.direction.header.frame_id = "base_link"
    # place.post_grasp_retreat.direction.vector.z = 1.0
    # place.post_grasp_retreat.min_distance = 0.01
    # place.post_grasp_retreat.desired_distance = 0.05

    # place.max_contact_force = 10

    # place.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
    # traj = JointTrajectoryPoint()
    # traj.positions.append(0.4)
    # traj.time_from_start = rospy.Duration.from_sec(0.5)
    # place.pre_grasp_posture.points.append(traj)

    # place.grasp_posture.joint_names.append("gripper_finger1_joint")
    # traj = JointTrajectoryPoint()
    # traj.positions.append(0)
    # traj.time_from_start = rospy.Duration.from_sec(0.5)
    # place.grasp_posture.points.append(traj)

    # Setting place pose
    place.place_pose.header.frame_id = "base_link"
    roll,pitch,yaw,x,y,z = 0, pi/2, 0, 0.4, 0, 0.2
    quat = quaternion_from_euler(roll, pitch, yaw)
    place.place_pose.pose.orientation.x = quat[0]
    place.place_pose.pose.orientation.y = quat[1]
    place.place_pose.pose.orientation.z = quat[2]
    place.place_pose.pose.orientation.w = quat[3]
    place.place_pose.pose.position.x = x
    place.place_pose.pose.position.y = y
    place.place_pose.pose.position.z = z

    # Setting pre-place approach
    place.pre_place_approach.direction.header.frame_id = "base_link"
    place.pre_place_approach.direction.vector.z = -0.2
    place.pre_place_approach.min_distance = 0.01
    place.pre_place_approach.desired_distance = 0.05

    # Setting post-place retreat
    place.post_place_retreat.direction.header.frame_id = "base_link"
    place.post_place_retreat.direction.vector.z = 0.2
    place.post_place_retreat.min_distance = 0.01
    place.post_place_retreat.desired_distance = 0.05

    place.post_place_posture.joint_names.append("gripper_finger1_joint")
    traj = JointTrajectoryPoint()
    traj.positions.append(0)
    traj.time_from_start = rospy.Duration.from_sec(0.5)
    place.post_place_posture.points.append(traj)

    #openGripper()
    


if __name__ == "__main__":
    setting_grasp()
    setting_place()

    box_name = "box"
    x, y, z = 0.5, 0, 0.015
    size_x, size_y, size_z= 0.03, 0.03, 0.03
    scene.addBox(box_name, size_x, size_y, size_z, x, y, z)
    scene.setColor(box_name, 1, 0, 0)
    scene.sendColors()

    rospy.loginfo("Opening gripper")	
    move_joint_hand(0)
    rospy.sleep(1)

    # Move to HOME before start
    rospy.loginfo("Moving arm to HOME point")	
    move_pose_arm(0,1.57,0,0.4,0,0.6)
    rospy.sleep(1)

    rospy.loginfo("Opening gripper")	
    move_joint_hand(0)
    rospy.sleep(1)	
    
    rospy.loginfo("Picking Box")	
    pick_and_place.pickup(box_name, [grasp])
    rospy.sleep(1)

    rospy.loginfo("Placing Box")
    pick_and_place.place(box_name, [place])
    rospy.sleep(1)

    rospy.loginfo("Moving arm to HOME point")	
    move_pose_arm(0,0.8,0,0.4,0,0.6)
    rospy.sleep(1)

    scene.removeAttachedObject(box_name)

    moveit_commander.roscpp_shutdown()
