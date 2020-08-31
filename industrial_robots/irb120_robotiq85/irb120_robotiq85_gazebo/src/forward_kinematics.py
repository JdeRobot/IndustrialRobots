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

# Init stuff
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_irb120_robot', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("irb_120")
hand_group = moveit_commander.MoveGroupCommander("robotiq_85")

# Publish trajectory in RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Forward Kinematics (FK): move the arm by axis values
def move_joint_arm(joint_0,joint_1,joint_2,joint_3,joint_4,joint_5):
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = joint_0
    joint_goal[1] = joint_1
    joint_goal[2] = joint_2
    joint_goal[3] = joint_3
    joint_goal[4] = joint_4
    joint_goal[5] = joint_5

    arm_group.go(joint_goal, wait=True)
    arm_group.stop() # To guarantee no residual movement

# Move the Robotiq gripper by master axis
def move_joint_hand(gripper_finger1_joint):
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[2] = gripper_finger1_joint # Gripper master axis

    hand_group.go(joint_goal, wait=True)
    hand_group.stop() # To guarantee no residual movement


if __name__ == '__main__':
    
    # Print current robot state
    print "============ Printing robot state ============"
    print robot.get_current_state()
    print ""  

    # Example of FK in a loop:
    for i in range(2):	
	rospy.loginfo("Moving arm to pose_1")	
	move_joint_arm(-pi/2,0.5,0.2,-1,0.2,0.2)
        rospy.sleep(1)
	rospy.loginfo("Opening gripper")	
	move_joint_hand(0)
        rospy.sleep(1)
	rospy.loginfo("Moving arm to pose_2")
	move_joint_arm(0,0,-0.2,0,1,0.3)
        rospy.sleep(1)
        rospy.loginfo("Closing gripper")	
	move_joint_hand(0.5)
        rospy.sleep(1)

    rospy.loginfo("All movements finished. Shutting down")	
    moveit_commander.roscpp_shutdown()
