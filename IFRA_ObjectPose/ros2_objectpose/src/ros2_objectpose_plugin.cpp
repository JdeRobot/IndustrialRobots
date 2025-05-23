/*

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ObjectPose Plugin for ROS2-Gazebo Simulation. URL: https://github.com/IFRA-Cranfield/IFRA_ObjectPose.

*/

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_objectpose/ros2_objectpose_plugin.hpp"        // Header file.
#include <objectpose_msgs/msg/object_pose.hpp>                // ROS2 Message.

#include <memory>

namespace gazebo_ros
{

class ROS2ObjectPosePluginPrivate
{
public:

  // ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  // MODEL -> OBJECT:
  gazebo::physics::ModelPtr model_;
  
  // PUBLISH ObjectPose:
  void PublishStatus();                                                          // Method to publish ObjectPose.
  rclcpp::Publisher<objectpose_msgs::msg::ObjectPose>::SharedPtr pose_pub_;      // Publisher.
  objectpose_msgs::msg::ObjectPose pose_msg_;                                    // ObjectPose.

  // WORLD UPDATE event:
  void OnUpdate();
  rclcpp::Time last_publish_time_;
  int update_ns_;
  gazebo::event::ConnectionPtr update_connection_;  // Connection to world update event. Callback is called while this is alive.

};

ROS2ObjectPosePlugin::ROS2ObjectPosePlugin()
: impl_(std::make_unique<ROS2ObjectPosePluginPrivate>())
{
}

ROS2ObjectPosePlugin::~ROS2ObjectPosePlugin()
{
}

void ROS2ObjectPosePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  
  // Create ROS2 node:
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // OBTAIN -> MODEL (OBJECT):
  impl_->model_ = _model;

	// ModelName:
	std::string modelname = impl_->model_->GetName();
	impl_->pose_msg_.objectname = modelname;

  // Create ObjectPose publisher:
  impl_->pose_pub_ = impl_->ros_node_->create_publisher<objectpose_msgs::msg::ObjectPose>("ObjectPose", 10);

  double publish_rate = 100.0;
  impl_->update_ns_ = int((1/publish_rate) * 1e9);
  impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

  // Create a connection so the OnUpdate function is called at every simulation iteration: 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ROS2ObjectPosePluginPrivate::OnUpdate, impl_.get()));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO ObjectPose plugin loaded successfully.");
}

void ROS2ObjectPosePluginPrivate::OnUpdate()
{
  
	// GET ObjectPose from GAZEBO API, and assign it to pose_msg_:
	ignition::math::Pose3d CurrentPose = model_-> WorldPose();

	// ASSIGN VALUES:
	pose_msg_.x = CurrentPose.Pos().X();
	pose_msg_.y = CurrentPose.Pos().Y();
	pose_msg_.z = CurrentPose.Pos().Z();
	pose_msg_.qx = CurrentPose.Rot().X();
	pose_msg_.qy = CurrentPose.Rot().Y();
	pose_msg_.qz = CurrentPose.Rot().Z();
  pose_msg_.qw = CurrentPose.Rot().W();

  PublishStatus();

  // This section below was blocking the pose publishing of >+1 objects in Gazebo (unknown reason, probably related to get_clock).
  // It has been removed, and now the object pose is published in every single simulation iteration --> OnUpdate!

  // Publish status at rate:
  //rclcpp::Time now = ros_node_->get_clock()->now();
  //if (now - last_publish_time_ >= rclcpp::Duration(0, update_ns_)) {
  //  PublishStatus();
  //  last_publish_time_ = now;
  //}
    
}

void ROS2ObjectPosePluginPrivate::PublishStatus(){
  
  pose_pub_->publish(pose_msg_);

}

GZ_REGISTER_MODEL_PLUGIN(ROS2ObjectPosePlugin)
}  // namespace gazebo_ros