#! /usr/bin/env python

import rospy
import rospkg

import sys
import copy
import os
import numpy

from moveit_commander import RobotCommander,PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion


class Pick_Place:
    def __init__ (self):
        self.objects_name = ["cylinder", "box", "sphere"]
        self.object_width = 0.03

        self.arm = moveit_commander.MoveGroupCommander("irb_120")
        self.gripper = moveit_commander.MoveGroupCommander("robotiq_85")

        self.arm.set_goal_tolerance(0.2)
        self.gripper.set_goal_tolerance(0.05)

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.add_ground()

        rospy.sleep(1.0)

        for i in range(3):
            rospy.loginfo("Moving arm to HOME point")	
            self.move_pose_arm(0,1.57,0,0.4,0,0.6)
            rospy.sleep(1)

            self.object_name = self.objects_name[i]
            self.scene.remove_world_object(self.object_name)
            self.pose_object, dx, dy, dz = self.add_object(self.object_name)
            rospy.sleep(1.0)

            self.pose_object.position.x += dx
            self.pose_object.position.y += dy
            self.pose_object.position.z += dz
            print(self.objects_name[i],self.pose_object.position)

            self.approach_retreat_desired_dist = 0.2
            self.approach_retreat_min_dist = 0.1

            print("start pick and place")

            # Pick
            grasps = self.generate_grasps(self.object_name, self.pose_object)
            self.arm.pick(self.object_name, grasps)
            self.gripper.stop()

            rospy.loginfo('Pick up successfully')
            self.arm.detach_object(self.object_name)
            self.clean_scene(self.object_name)
            rospy.sleep(1)

            curr_pose = self.arm.get_current_pose().pose
            x = curr_pose.position.x
            y = curr_pose.position.y
            z = curr_pose.position.z
            # quaternion = (curr_pose.orientation.x,
            #             curr_pose.orientation.y,
            #             curr_pose.orientation.z,
            #             curr_pose.orientation.w)
            # euler = euler_from_quaternion(quaternion)
            # roll = euler[0]
            # pitch = euler[1]
            # yaw = euler[2]
            roll = 0.0
            pitch = numpy.deg2rad(90.0)
            yaw = 0.0

            z += 0.1
            self.move_pose_arm(roll, pitch, yaw, x, y, z)

            x = -curr_pose.position.y
            y = 0.6
            z -= 0.1
            #z = self.pose_object.position.z+0.05
            # if self.object_name == "cylinder":
            #     yaw = numpy.deg2rad(90.0)
                #z+= 0.001

            self.move_pose_arm(roll, pitch, yaw, x, y, z)
            rospy.sleep(0.5)

            z -= 0.1
            self.move_pose_arm(roll, pitch, yaw, x, y, z)
            self.move_joint_hand(0)
            
            #if self.object_name == "cylinder":
            z += 0.1
            self.move_pose_arm(roll, pitch, yaw, x, y, z)

            # target_pose = curr_pose
            # target_pose.position.x = x
            # target_pose.position.y = y
            # target_pose.position.z = z
            # q = quaternion_from_euler(roll, pitch, yaw)
            # target_pose.orientation = Quaternion(*q)

            # place = self.generate_places(target_pose)
            # self.arm.place(self.object_name, place)

            # self.arm.detach_object(self.object_name)
            # self.clean_scene(self.object_name)

            rospy.loginfo('Place successfully')

            rospy.loginfo("Moving arm to HOME point")	
            self.move_pose_arm(0,0.8,0,0.4,0,0.6)
            rospy.sleep(1)


    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def add_ground(self):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.01
        size = (5,5,0.02)

        self.scene.add_box("ground",p, size)

    def add_object(self, name):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        dx = 0
        dy = 0
        dz = 0

        if name == "box":
            p.pose.position.x = 0.5
            p.pose.position.y = 0.0
            p.pose.position.z = 0.015+0.115

            q = quaternion_from_euler(0.0, 0.0, 0.0)
            p.pose.orientation = Quaternion(*q)
            size = (0.03,0.03,0.03)

            self.scene.add_box(name, p, size)
            dz = 0.16

        elif name == "cylinder":
            p.pose.position.x = 0.5
            p.pose.position.y = 0.2
            p.pose.position.z = 0.05+0.115

            q = quaternion_from_euler(0.0, 0.0, 0.0)
            p.pose.orientation = Quaternion(*q)

            height = 0.1
            radius = 0.03

            self.scene.add_cylinder(name, p, height, radius)
            dz = 0.16
            #dx = -0.135

        elif name == "sphere":
            p.pose.position.x = 0.5
            p.pose.position.y = -0.2
            p.pose.position.z = 0.03+0.115

            q = quaternion_from_euler(0.0, 0.0, 0.0)
            p.pose.orientation = Quaternion(*q)

            radius = 0.03

            self.scene.add_sphere(name, p, radius)
            dz = 0.15

        return p.pose, dx, dy, dz

    def move_pose_arm(self, roll,pitch,yaw,x,y,z):
        pose_goal = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(roll,pitch,yaw)
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

        # Move the Robotiq gripper by master axis
    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement

    def generate_grasps(self, name, pose):
        grasps = []

        now = rospy.Time.now()
        #for angle in numpy.arange(0.0, numpy.deg2rad(360.0), numpy.deg2rad(10.0)):
            # Create place location:
        angle = 0
        grasp = Grasp()

        grasp.grasp_pose.header.stamp = now
        grasp.grasp_pose.header.frame_id = self.robot.get_planning_frame()
        grasp.grasp_pose.pose = copy.deepcopy(pose)

        # Setting pre-grasp approach
        grasp.pre_grasp_approach.direction.header.stamp = now
        grasp.pre_grasp_approach.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.pre_grasp_approach.direction.vector.z = -0.2
        grasp.pre_grasp_approach.min_distance = self.approach_retreat_min_dist
        grasp.pre_grasp_approach.desired_distance = self.approach_retreat_desired_dist

        # Setting post-grasp retreat
        grasp.post_grasp_retreat.direction.header.stamp = now
        grasp.post_grasp_retreat.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.post_grasp_retreat.direction.vector.z = 0.2
        grasp.post_grasp_retreat.min_distance = self.approach_retreat_min_dist
        grasp.post_grasp_retreat.desired_distance = self.approach_retreat_desired_dist


        # if name != "cylinder":
        q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), angle)
        grasp.grasp_pose.pose.orientation = Quaternion(*q)
        # else:
        #     #grasp.pre_grasp_approach.direction.vector.z = -0.05
        #     grasp.pre_grasp_approach.direction.vector.x = 0.1
        #     #grasp.post_grasp_retreat.direction.vector.z = 0.05
        #     grasp.post_grasp_retreat.direction.vector.x = -0.1

        grasp.max_contact_force = 100

        grasp.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(0.03)
        traj.time_from_start = rospy.Duration.from_sec(0.5)
        grasp.pre_grasp_posture.points.append(traj)

        grasp.grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        if name == "box":
            traj.positions.append(0.57)
        elif name == "sphere":
            traj.positions.append(0.3)
        else:
            traj.positions.append(0.3)

        #traj.velocities.append(0.2)
        traj.effort.append(800)
        traj.time_from_start = rospy.Duration.from_sec(5.0)
        grasp.grasp_posture.points.append(traj)

        grasps.append(grasp)

        return grasps

    def generate_places(self, target):
        #places = []
        now = rospy.Time.now()
        # for angle in numpy.arange(0.0, numpy.deg2rad(360.0), numpy.deg2rad(30.0)):
            # Create place location:
        place = PlaceLocation()

        place.place_pose.header.stamp = now
        place.place_pose.header.frame_id = self.robot.get_planning_frame()

        # Set target position:
        place.place_pose.pose = copy.deepcopy(target)

        # Generate orientation (wrt Z axis):
        # q = quaternion_from_euler(0.0, 0, 0.0)
        # place.place_pose.pose.orientation = Quaternion(*q)

        # Generate pre place approach:
        place.pre_place_approach.desired_distance = self.approach_retreat_desired_dist
        place.pre_place_approach.min_distance = self.approach_retreat_min_dist

        place.pre_place_approach.direction.header.stamp = now
        place.pre_place_approach.direction.header.frame_id = self.robot.get_planning_frame()

        place.pre_place_approach.direction.vector.x =  0
        place.pre_place_approach.direction.vector.y =  0
        place.pre_place_approach.direction.vector.z = -0.2

        # Generate post place approach:
        place.post_place_retreat.direction.header.stamp = now
        place.post_place_retreat.direction.header.frame_id = self.robot.get_planning_frame()

        place.post_place_retreat.desired_distance = self.approach_retreat_desired_dist
        place.post_place_retreat.min_distance = self.approach_retreat_min_dist

        place.post_place_retreat.direction.vector.x = 0
        place.post_place_retreat.direction.vector.y = 0
        place.post_place_retreat.direction.vector.z = 0.2

        place.allowed_touch_objects.append(self.object_name)

        place.post_place_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(0)
        #traj.effort.append(0)
        traj.time_from_start = rospy.Duration.from_sec(1.0)
        place.post_place_posture.points.append(traj)

            # Add place:
            #places.append(place)

        return place

if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')

    p = Pick_Place()

    rospy.spin()
    roscpp_shutdown()