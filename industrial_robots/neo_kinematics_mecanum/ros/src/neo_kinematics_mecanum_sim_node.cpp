/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distributionh.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permissionh.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../../common/include/Kinematics.h"
#include "../../common/include/MecanumKinematics.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<std_msgs/Float64.h>


class NeoMecKinSimNode
{
public:
	NeoMecKinSimNode();
	virtual ~NeoMecKinSimNode();

	// ROS-Node handle 
	ros::NodeHandle nh;

	// Subscribers 
	ros::Subscriber topicSub_GazeboLinkState;
	ros::Subscriber topicSub_ComVel;
	ros::Subscriber topicsub_Joint_States;

	// Publishers
	ros::Publisher wheel0;
	ros::Publisher wheel1;
	ros::Publisher wheel2;
	ros::Publisher wheel3;
	ros::Publisher topicPub_Odometry;
	ros::Publisher topicPub_DriveCommands;

	// Message declarations
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::Twist twist;
	ros::Time joint_state_odom_stamp_;
	tf::TransformBroadcaster odom_broadcaster;

	// int init();
	void receiveCmd(const geometry_msgs::Twist& msg);
	void HeaderStampCB(const sensor_msgs::JointState::ConstPtr& msg);

private:
	boost::mutex mutex;
	Mecanum4WKinematics* kin = 0;
	OdomPose pose;
	bool sendTransform = false;
};



NeoMecKinSimNode::NeoMecKinSimNode()
{
	kin = new Mecanum4WKinematics();

	pose.xAbs = 0;
	pose.yAbs = 0;
	pose.phiAbs = 0;

	double wheelDiameter, axisWidth, axisLength;
	double devX, devY, devZ, devRoll, devPitch, devYaw;

	nh.param("wheelDiameter", wheelDiameter, 0.3);
	nh.param("robotWidth", axisWidth, 0.5);
	nh.param("robotLength", axisLength, 0.5);
	nh.param("devX", devX, 0.1);
	nh.param("devY", devY, 0.1);
	nh.param("devZ", devZ, 0.1);
	nh.param("devRoll", devRoll, 0.1);
	nh.param("devPitch", devPitch, 0.1);
	nh.param("devYaw", devYaw, 0.1);
	nh.param<bool>("sendTransform", sendTransform, false);
	kin->setWheelDiameter(wheelDiameter);
	kin->setAxis1Length(axisWidth);
	kin->setAxis2Length(axisLength);

	if(sendTransform) {
        ROS_INFO("neo_kinematics_mecanum_sim_node: sending transformation");
	} else {
        ROS_INFO("neo_kinematics_mecanum_sim_node: sending no transformation");
	}

	kin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
	
	// Subscribe to joint_states 
	topicsub_Joint_States = nh.subscribe("/joint_states", 1, &NeoMecKinSimNode::HeaderStampCB, this); 

	// Subscribe to cmd_vel for the converstion to joint velocity
	topicSub_ComVel = nh.subscribe("/cmd_vel", 1, &NeoMecKinSimNode::receiveCmd, this);

	// Virtual controllers created for simulating the wheels  
	wheel0 = nh.advertise<std_msgs::Float64>("/mpo_500_omni_wheel_back_left_controller/command", 1);
	wheel1 = nh.advertise<std_msgs::Float64>("/mpo_500_omni_wheel_back_right_controller/command", 1);
	wheel2 = nh.advertise<std_msgs::Float64>("/mpo_500_omni_wheel_front_left_controller/command", 1);
	wheel3 = nh.advertise<std_msgs::Float64>("/mpo_500_omni_wheel_front_right_controller/command", 1);
}

NeoMecKinSimNode::~NeoMecKinSimNode()
{}


void NeoMecKinSimNode::HeaderStampCB(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint_state_odom_stamp_ = msg->header.stamp;
}

void NeoMecKinSimNode::receiveCmd(const geometry_msgs::Twist& msg)
{
	twist = msg;
	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist, traj);
	std_msgs::Float64 f1,f2,f3,f4;
	f1.data = traj.points[0].velocities[0];
	f2.data = -traj.points[0].velocities[1];
	f3.data = traj.points[0].velocities[2];
	f4.data = -traj.points[0].velocities[3];

	wheel0.publish(f1);
	wheel1.publish(f2);
	wheel2.publish(f3);
	wheel3.publish(f4);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "neo_kinematics_mecanum_sim_node");

	NeoMecKinSimNode node;

	ros::Rate loop_rate(100); 
	while (ros::ok())
   	{
      loop_rate.sleep();   
      ros::spinOnce();   		
   	}  
	return 0;
}