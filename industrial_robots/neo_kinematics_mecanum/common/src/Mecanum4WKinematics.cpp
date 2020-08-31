/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
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


#include "../include/MecanumKinematics.h"

#include <geometry_msgs/Vector3.h>


Mecanum4WKinematics::Mecanum4WKinematics()
{
	m_dAxis1Length = 0.5;
	m_dAxis2Length = 0.7;
	m_dDiam = 0.3;
	m_dStdDevX = 0.1;
	m_dStdDevY = 0.1;
	m_dStdDevZ = 0.1;
	m_dStdDevRoll = 0.1;
	m_dStdDevPitch = 0.1;
	m_dStdDevYaw = 0.1;
}

void Mecanum4WKinematics::execForwKin(const sensor_msgs::JointState& js, nav_msgs::Odometry& odom, OdomPose& cpose)
{
	current_time = js.header.stamp;

	/* See "Omnidirectional Mobile Robot -Design and Implementation": Ioan Doroftei, Victor Grosu and Veaceslav Spinu

		l = l1 + l2

		             |                      |   | w1 |
		| vx |       |  1     1    1    1   | . | w2 |
		| vy | = R/4 | -1     1    1   -1   |   | w3 |
		| wz |       | -1/l  1/l -1/l  1/l  |   | w4 |
	*/

	//velocities:
	const double move_vel_x = (js.velocity[0] + js.velocity[1] + js.velocity[2] + js.velocity[3]) * m_dDiam / 8;
	const double move_vel_y = (js.velocity[1] - js.velocity[0] - js.velocity[3] + js.velocity[2]) * m_dDiam / 8;
	const double move_yawrate = (-js.velocity[0] + js.velocity[1] - js.velocity[2] + js.velocity[3]) * m_dDiam / 4
									/ (m_dAxis1Length + m_dAxis2Length);

	//positions:
	if(!last_time.is_zero())
	{
		double dt = (current_time - last_time).toSec();

		// check for valid delta time
		if(dt > 0 && dt < 1)
		{
			// compute second order midpoint velocities
			const double vel_x_mid = 0.5 * (move_vel_x + last_odom.twist.twist.linear.x);
			const double vel_y_mid = 0.5 * (move_vel_y + last_odom.twist.twist.linear.y);
			const double yawrate_mid = 0.5 * (move_yawrate + last_odom.twist.twist.angular.z);

			// compute midpoint yaw angle
			const double yaw_mid = cpose.phiAbs + 0.5 * yawrate_mid * dt;

			// integrate position using midpoint velocities and yaw angle
			cpose.xAbs += vel_x_mid * dt * cos(yaw_mid) + vel_y_mid * dt * -sin(yaw_mid);
			cpose.yAbs += vel_x_mid * dt * sin(yaw_mid) + vel_y_mid * dt * cos(yaw_mid);

			// integrate yaw angle using midpoint yawrate
			cpose.phiAbs += yawrate_mid * dt;
		}
		else
		{
			ROS_WARN_STREAM("invalid joint state delta time: " << dt << " sec");
		}
	}

	odom.twist.twist.linear.x = move_vel_x;
	odom.twist.twist.linear.y = move_vel_y;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = move_yawrate;

	//define cov for twist msg
	odom.twist.covariance[0] = m_dStdDevX * m_dStdDevX;
	odom.twist.covariance[7] = m_dStdDevY * m_dStdDevY;
	odom.twist.covariance[14] = m_dStdDevZ * m_dStdDevZ;
	odom.twist.covariance[21] = m_dStdDevRoll * m_dStdDevRoll;
	odom.twist.covariance[28] = m_dStdDevPitch * m_dStdDevPitch;
	odom.twist.covariance[35] = m_dStdDevYaw * m_dStdDevYaw;

	odom.pose.pose.position.x = cpose.xAbs;
	odom.pose.pose.position.y = cpose.yAbs;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cpose.phiAbs);

	last_time = current_time;
	last_odom = odom;
}

void Mecanum4WKinematics::execInvKin(const geometry_msgs::Twist& twist, trajectory_msgs::JointTrajectory& traj)
{
	//make sure that there's no old command in the message.
	traj.joint_names.clear();
	traj.points.clear();
	/* See "Omnidirectional Mobile Robot -Design and Implementation": Ioan Doroftei, Victor Grosu and Veaceslav Spinu

		| w1 |       | 1 -1 -(l1+l2) |   
		| w2 | = 1/R | 1  1  (l1+l2) | . | vx |
		| w3 |       | 1  1 -(l1+l2) |   | vy |
		| w4 |       | 1 -1  (l1+l2) |   | wz |
	*/
	trajectory_msgs::JointTrajectoryPoint point;
	point.velocities.resize(4);
	// w1:
	traj.joint_names.push_back("mpo_500_omni_wheel_front_left_joint");
	point.velocities[0] = 2 / m_dDiam * ( twist.linear.x - twist.linear.y - (m_dAxis1Length + m_dAxis2Length) / 2 * twist.angular.z);
	// w2:
	traj.joint_names.push_back("mpo_500_omni_wheel_front_right_joint");
	point.velocities[1] = 2 / m_dDiam * ( twist.linear.x + twist.linear.y + (m_dAxis1Length + m_dAxis2Length) / 2 * twist.angular.z);
	// w3:
	traj.joint_names.push_back("mpo_500_omni_wheel_back_left_joint");
	point.velocities[2] = 2 / m_dDiam * ( twist.linear.x + twist.linear.y - (m_dAxis1Length + m_dAxis2Length) / 2 * twist.angular.z);
	// w4:
	traj.joint_names.push_back("mpo_500_omni_wheel_back_right_joint");
	point.velocities[3] = 2 / m_dDiam * ( twist.linear.x - twist.linear.y + (m_dAxis1Length + m_dAxis2Length) / 2 * twist.angular.z);
	traj.points.push_back(point);

}



void Mecanum4WKinematics::setAxis1Length(double dLength)
{
	m_dAxis1Length = dLength;
}

void Mecanum4WKinematics::setAxis2Length(double dLength)
{
	m_dAxis2Length = dLength;
}


void Mecanum4WKinematics::setWheelDiameter(double dDiam)
{
	m_dDiam = dDiam;
}

void Mecanum4WKinematics::setStdDev(double dStdDevX, double dStdDevY, double dStdDevZ, double dStdDevRoll, double dStdDevPitch, double dStdDevYaw)
{
	m_dStdDevX = dStdDevX;
	m_dStdDevY = dStdDevY;
	m_dStdDevZ = dStdDevZ;
	m_dStdDevRoll = dStdDevRoll;
	m_dStdDevPitch = dStdDevPitch;
	m_dStdDevYaw = dStdDevYaw;
}
