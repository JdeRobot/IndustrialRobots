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
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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


class PlatformCtrlNode 
{
public:
	virtual ~PlatformCtrlNode();

	ros::NodeHandle nh;
	ros::Publisher topicPub_Odometry;	
	ros::Subscriber topicSub_DriveState;
	ros::Publisher topicPub_DriveCommands;	
	ros::Subscriber topicSub_ComVel;
	tf::TransformBroadcaster odom_broadcaster;

	int init();
	void receiveCmd(const geometry_msgs::Twist& twist);
	void sendOdom(const sensor_msgs::JointState& js);

private:
	boost::mutex mutex;
	OdomPose pose;
	Mecanum4WKinematics* kin = 0;
	bool sendTransform = false;

};

PlatformCtrlNode::~PlatformCtrlNode()
{
	delete kin;
}

int PlatformCtrlNode::init()
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
        ROS_INFO("neo_kinematics_mecanum_node: sending transformation");
	} else {
        ROS_INFO("neo_kinematics_mecanum_node: sending no transformation");
	}

	kin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
	
	topicPub_Odometry = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    topicSub_DriveState = nh.subscribe("/drives/joint_states", 1, &PlatformCtrlNode::sendOdom, this);
    topicPub_DriveCommands = nh.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory", 1);
	topicSub_ComVel = nh.subscribe("/cmd_vel", 1, &PlatformCtrlNode::receiveCmd, this);
	return 0;
}

void PlatformCtrlNode::receiveCmd(const geometry_msgs::Twist& twist)
{
	boost::lock_guard<boost::mutex> lock(mutex);

	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist, traj);
	topicPub_DriveCommands.publish(traj);
}

void PlatformCtrlNode::sendOdom(const sensor_msgs::JointState& js)
{
	boost::lock_guard<boost::mutex> lock(mutex);

    //check if js has data from 4 motors
	if(js.velocity.size() < 4) {
		return;
	}

	//odometry msg
	nav_msgs::Odometry odom;
	odom.header.stamp = js.header.stamp;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	kin->execForwKin(js, odom, pose);
	topicPub_Odometry.publish(odom);

	//odometry transform:
	if(sendTransform)
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odom.header.frame_id;
		odom_trans.child_frame_id = odom.child_frame_id;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster.sendTransform(odom_trans);
	}
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "neo_kinematics_mecanum_node");

	PlatformCtrlNode node;
    if(node.init() != 0) {
    	ROS_ERROR("neo_kinematics_mecanum_node: init failed!");
    }

	ros::spin();

	return 0;
}

