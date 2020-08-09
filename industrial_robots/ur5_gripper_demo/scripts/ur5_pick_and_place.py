#!/usr/bin/env python
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_gripper_demo.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
tracker = Tracker()

class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        #self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        #rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('ur5')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Inverse Kinematics (IK): move TCP to given position and orientation
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

        plan = self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

    def move_to_home(self):
        # Move to HOME before start
        rospy.loginfo("Moving arm to HOME point")	
        self.move_pose_arm(0,1.57,0,0.4,0,0.6)
        rospy.sleep(1)

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def pick(self):
        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose().pose

        # Initialize the waypoints list
        self.waypoints= []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)
        #self.waypoints.append(deepcopy(wpose))

        wpose.position.x = 0
        wpose.position.y = -0.8
        wpose.position.z = 0.1
        self.waypoints.append(deepcopy(wpose))

        wpose.position.z = -0.1+0.03
        self.waypoints.append(deepcopy(wpose))


        self.arm.set_start_state_to_current_state()
        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
        rospy.loginfo("Cartesian path planned")	

        self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.loginfo("Arrive picking pose")

        tracker.flag2 = 1
        self.cxy_pub.publish(tracker)

    def place(self):
        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose().pose

        # Initialize the waypoints list
        self.waypoints= []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)
        #self.waypoints.append(deepcopy(wpose))

        wpose.position.x = 0.3
        wpose.position.y = -0.1
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        wpose.position.z = 0.0
        self.waypoints.append(deepcopy(wpose))


        self.arm.set_start_state_to_current_state()
        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
        rospy.loginfo("Cartesian path planned")	

        self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.loginfo("Arrive placing pose")

        tracker.flag2 = 0
        self.cxy_pub.publish(tracker)

if __name__=="__main__":
    mp=ur5_mp()

    # mp.move_to_home()
    # mp.pick()
    #rospy.sleep(10)
    # mp.place()

    rospy.spin()