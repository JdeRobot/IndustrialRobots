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


#ifndef neo_diffdrivekinematics_h_
#define neo_diffdrivekinematics_h_

#include "Kinematics.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>

class Mecanum4WKinematics : public Kinematics {
public:
	Mecanum4WKinematics();
	void execForwKin(const sensor_msgs::JointState& js, nav_msgs::Odometry& odom, OdomPose& cpose);
	void execInvKin(const geometry_msgs::Twist& twist, trajectory_msgs::JointTrajectory& traj);
	/*
	 robot:	l1: m_dAxis1Length
		l2: m_dAxis2Length

	      --|##2##|        |##4##|
	      ^   ##################
	  l1  ¦   ##################             ^ y
	      ¦   ##################             ¦
	      v   ##################       x     ¦
	      --|##1##|        |##3##|     <-----¦-
		   |       l2     |
		   |<------------>|
	*/
	void setAxis1Length(double dLength);
	void setAxis2Length(double dLength);
	void setWheelDiameter(double dDiam);
	void setStdDev(double dStdDevX, double dStdDevY, double dStdDevZ, double dStdDevRoll, double dStdDevPitch, double dStdDevYaw);

private:
	double m_dAxis1Length;
	double m_dAxis2Length;
	double m_dDiam;
	double m_dStdDevX;
	double m_dStdDevY;
	double m_dStdDevZ;
	double m_dStdDevRoll;
	double m_dStdDevPitch;
	double m_dStdDevYaw;
};


#endif //neo_diffdrivekinematics_h_
