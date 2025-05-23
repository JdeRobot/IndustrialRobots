/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <cmath>
#include <stdexcept>

#include "abb_egm_rws_managers/egm_manager.h"

namespace
{
/**
 * \brief Number of allowed missed messages, before a channel is assumed to be inactive.
 */
constexpr unsigned int MISSED_MESSAGES_THRESHOLD{5};

/**
 * \brief Minimum accepted speed [deg/s or mm/s] for velocity commands.
 *
 * Intended for filtering away numerical errors.
 */
constexpr double MIN_SPEED_THRESHOLD{0.5};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Class definitions: EGMManager::Channel
 */

/***********************************************************
 * Primary methods
 */

EGMManager::Channel::Channel(const ChannelConfiguration& configuration,
                             boost::asio::io_service& io_service,
                             boost::shared_ptr<boost::condition_variable> p_new_message_cv)
:
configuration_{configuration},
channel_is_active_{false},
missed_messages_{MISSED_MESSAGES_THRESHOLD}
{
  egm::BaseConfiguration interface_cfg{};
  interface_cfg.use_velocity_outputs = true;
  interface_cfg.p_new_message_cv = p_new_message_cv;

  // Check for special cases (six axes robot is used by default).
  if(configuration.mech_unit_group.has_robot())
  {
    if(configuration.mech_unit_group.robot().axes_total() == 7)
    {
      interface_cfg.axes = egm::RobotAxes::Seven;
    }
  }
  else
  {
    interface_cfg.axes = egm::RobotAxes::None;
  }

  // Create the communication interface.
  p_interface_ = std::make_unique<egm::EGMControllerInterface>(io_service, configuration.port_number, interface_cfg);

  if(!p_interface_ || !p_interface_->isInitialized())
  {
    throw std::runtime_error{"Failed to initialize EGM interface"};
  }
}

bool EGMManager::Channel::read(MotionData::MechanicalUnitGroup& group)
{
  bool any_new_states{false};

  p_interface_->read(&input_);

  // Check the sequence number to detect missed messages.
  if(input_.header().sequence_number() == previous_header_.sequence_number())
  {
    ++missed_messages_;
  }
  else
  {
    missed_messages_ = 0;
  }

  // Check if the channel is active.
  channel_is_active_ = missed_messages_ < MISSED_MESSAGES_THRESHOLD;

  // Keep track of:
  // - Activation/deactivation of the channel.
  // - If the channel is currently active.
  // - If the channel's RAPID execution status has changed.
  // - If the channel's EGM client state has changed.
  group.egm_channel_data.was_activated_or_deactivated = group.egm_channel_data.is_active != channel_is_active_;
  group.egm_channel_data.is_active = channel_is_active_;
  group.egm_channel_data.rapid_execution_status_changed = input_.status().rapid_execution_state() !=
                                                          previous_status_.rapid_execution_state();
  group.egm_channel_data.egm_client_state_changed = input_.status().egm_state() != previous_status_.egm_state();

  // Update the states if the channel is active.
  if(channel_is_active_)
  {
    group.egm_channel_data.input.CopyFrom(input_);

    if(updateTCPRobotJointStates(group))
    {
      any_new_states = true;
    }

    if(updateExternalJointStates(group))
    {
      any_new_states = true;
    }
  }

  previous_header_.CopyFrom(input_.header());
  previous_status_.CopyFrom(input_.status());

  return any_new_states;
}

void EGMManager::Channel::write(const MotionData::MechanicalUnitGroup& group)
{
  // Write commands if the channel is active.
  if(channel_is_active_)
  {
    prepareOutputs();
    updateTCPRobotJointCommands(group);
    updateExternalJointCommands(group);
    p_interface_->write(output_);
  }
}

/***********************************************************
 * Auxiliary methods
 */

const std::string& EGMManager::Channel::getMechanicalUnitGroupName() const
{
  return configuration_.mech_unit_group.name();
}

int EGMManager::Channel::countActiveTCPRobotJoints(const MotionData::MechanicalUnitGroup& group)
{
  int count{0};

  for(const auto& unit : group.units)
  {
    if(unit.active && unit.type == MechanicalUnit_Type_TCP_ROBOT)
    {
      count += unit.joints.size();
    }
  }

  return count;
}

int EGMManager::Channel::countActiveExternalJoints(const MotionData::MechanicalUnitGroup& group)
{
  int count{0};

  for(const auto& unit : group.units)
  {
    if(unit.active && (unit.type == MechanicalUnit_Type_ROBOT || unit.type == MechanicalUnit_Type_SINGLE))
    {
      count += unit.joints.size();
    }
  }

  return count;
}

bool EGMManager::Channel::updateTCPRobotJointStates(MotionData::MechanicalUnitGroup& group)
{
  const auto& positions{input_.feedback().robot().joints().position()};
  const auto& velocities{input_.feedback().robot().joints().velocity()};

  const int active_joints{countActiveTCPRobotJoints(group)};

  if(active_joints != positions.values_size() || active_joints != velocities.values_size())
  {
    throw std::runtime_error{"Mismatch between number of assumed active robot joints and EGM feedback"};
  }

  if(active_joints == 0)
  {
    return false;
  }

  int counter{0};

  for(auto& unit : group.units)
  {
    if(unit.active && unit.type == MechanicalUnit_Type_TCP_ROBOT)
    {
      for(auto& joint : unit.joints)
      {
        double conversion_factor{joint.rotational ? Constants::DEG_TO_RAD : Constants::MM_TO_M};
        joint.state.position = positions.values(counter)*conversion_factor;
        joint.state.velocity = velocities.values(counter)*conversion_factor;
        ++counter;
      }
    }
  }

  return true;
}

bool EGMManager::Channel::updateExternalJointStates(MotionData::MechanicalUnitGroup& group)
{
  const auto& positions{input_.feedback().external().joints().position()};
  const auto& velocities{input_.feedback().external().joints().velocity()};

  const int active_joints{countActiveExternalJoints(group)};

  if(active_joints != positions.values_size() || active_joints != velocities.values_size())
  {
    throw std::runtime_error{"Mismatch between number of assumed active external joints and EGM feedback"};
  }

  if(active_joints == 0)
  {
    return false;
  }

  int counter{0};

  for(auto& unit : group.units)
  {
    if(unit.active && (unit.type == MechanicalUnit_Type_ROBOT || unit.type == MechanicalUnit_Type_SINGLE))
    {
      for(auto& joint : unit.joints)
      {
        double conversion_factor{joint.rotational ? Constants::DEG_TO_RAD : Constants::MM_TO_M};
        joint.state.position = positions.values(counter)*conversion_factor;
        joint.state.velocity = velocities.values(counter)*conversion_factor;
        ++counter;
      }
    }
  }

  return true;
}

void EGMManager::Channel::prepareOutputs()
{
  output_.mutable_robot()->CopyFrom(input_.feedback().robot());
  output_.mutable_external()->CopyFrom(input_.feedback().external());

  auto robot_cartesian_velocities{output_.mutable_robot()->mutable_cartesian()->mutable_velocity()};
  robot_cartesian_velocities->mutable_linear()->set_x(0.0);
  robot_cartesian_velocities->mutable_linear()->set_y(0.0);
  robot_cartesian_velocities->mutable_linear()->set_z(0.0);
  robot_cartesian_velocities->mutable_angular()->set_x(0.0);
  robot_cartesian_velocities->mutable_angular()->set_y(0.0);
  robot_cartesian_velocities->mutable_angular()->set_z(0.0);

  auto robot_joint_velocities{output_.mutable_robot()->mutable_joints()->mutable_velocity()};
  for(int i = 0; i < robot_joint_velocities->values_size(); ++i)
  {
    robot_joint_velocities->set_values(i, 0.0);
  }

  auto external_joint_velocities{output_.mutable_external()->mutable_joints()->mutable_velocity()};
  for(int i = 0; i < external_joint_velocities->values_size(); ++i)
  {
    external_joint_velocities->set_values(i, 0.0);
  }
}

void EGMManager::Channel::updateTCPRobotJointCommands(const MotionData::MechanicalUnitGroup& group)
{
  auto p_positions{output_.mutable_robot()->mutable_joints()->mutable_position()};
  auto p_velocities{output_.mutable_robot()->mutable_joints()->mutable_velocity()};

  int counter{0};

  for(auto& unit : group.units)
  {
    if(unit.active && unit.type == MechanicalUnit_Type_TCP_ROBOT)
    {
      for(const auto& joint : unit.joints)
      {
        // The validation throws an exception if any of the values are erroneous.
        validateJointCommand(joint);

        if(counter < p_positions->values_size() && counter < p_velocities->values_size())
        {
          double conversion_factor{(joint.rotational ? Constants::RAD_TO_DEG : Constants::M_TO_MM)};
          p_positions->set_values(counter, joint.command.position*conversion_factor);

          // Only accept velocity commands larger than the speed threshold.
          if(std::abs(joint.command.velocity*conversion_factor) > MIN_SPEED_THRESHOLD)
          {
            p_velocities->set_values(counter, joint.command.velocity*conversion_factor);
          }
        }

        ++counter;
      }
    }
  }
}

void EGMManager::Channel::updateExternalJointCommands(const MotionData::MechanicalUnitGroup& group)
{
  auto p_positions{output_.mutable_external()->mutable_joints()->mutable_position()};
  auto p_velocities{output_.mutable_external()->mutable_joints()->mutable_velocity()};

  int counter{0};

  for(auto& unit : group.units)
  {
    if(unit.active && (unit.type == MechanicalUnit_Type_ROBOT || unit.type == MechanicalUnit_Type_SINGLE))
    {
      for(const auto& joint : unit.joints)
      {
        // The validation throws an exception if any of the values are erroneous.
        validateJointCommand(joint);

        if(counter < p_positions->values_size() && counter < p_velocities->values_size())
        {
          double conversion_factor{(joint.rotational ? Constants::RAD_TO_DEG : Constants::M_TO_MM)};
          p_positions->set_values(counter, joint.command.position*conversion_factor);

          // Only accept velocity commands larger than the speed threshold.
          if(std::abs(joint.command.velocity*conversion_factor) > MIN_SPEED_THRESHOLD)
          {
            p_velocities->set_values(counter, joint.command.velocity*conversion_factor);
          }
        }

        ++counter;
      }
    }
  }
}

void EGMManager::Channel::validateJointCommand(const MotionData::Joint& joint)
{
  if(std::isnan(joint.command.position) || std::isnan(joint.command.velocity))
  {
    throw std::invalid_argument{"Joint command contains 'NaN' values"};
  }

  if(std::isinf(joint.command.position) || std::isinf(joint.command.velocity))
  {
    throw std::invalid_argument{"Joint command contains 'Inf' values"};
  }

  if(joint.command.position < joint.lower_limit || joint.command.position > joint.upper_limit)
  {
    throw std::out_of_range{"Joint command is out-of-bounds"};
  }
}

/***********************************************************************************************************************
 * Class definitions: EGMManager
 */

/***********************************************************
 * Primary methods
 */

EGMManager::EGMManager(const std::vector<ChannelConfiguration>& channel_configurations)
:
p_new_message_cv_{new boost::condition_variable{}}
{
  for(const auto& configuration : channel_configurations)
  {
    channels_.emplace_back(configuration, io_service_, p_new_message_cv_);
    threads_.emplace_back([&]{io_service_.run();});
  }
}

EGMManager::~EGMManager()
{
  io_service_.stop();

  for(auto& thread : threads_)
  {
    thread.join();
  }
}

bool EGMManager::waitForMessage(const unsigned int timeout_ms)
{
  boost::unique_lock<boost::mutex> lock{new_message_mutex_};

  if(timeout_ms == 0)
  {
    p_new_message_cv_->wait(lock);
    return true;
  }
  else
  {
    return p_new_message_cv_->timed_wait(lock, boost::posix_time::milliseconds(timeout_ms));
  }
}

bool EGMManager::read(MotionData& motion_data)
{
  bool any_new_states{false};

  for(auto& channel : channels_)
  {
    for(auto& motion_group : motion_data.groups)
    {
      if(motion_group.name == channel.getMechanicalUnitGroupName())
      {
        if(channel.read(motion_group))
        {
          any_new_states = true;
        }
      }
    }
  }

  return any_new_states;
}

void EGMManager::write(const MotionData& motion_data)
{
  for(auto& channel : channels_)
  {
    for(auto& motion_group : motion_data.groups)
    {
      if(motion_group.name == channel.getMechanicalUnitGroupName())
      {
        channel.write(motion_group);
      }
    }
  }
}

}
}
