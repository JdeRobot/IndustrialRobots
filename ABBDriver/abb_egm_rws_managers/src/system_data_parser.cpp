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

#include <algorithm>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

#include "abb_egm_rws_managers/system_data_parser.h"

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Class definitions: SystemDataParser
 */

/***********************************************************
 * Primary methods
 */

SystemDataParser::SystemDataParser(const SystemData& system_data, const std::string& prefix)
:
standalone_mechanical_unit_count_{0},
system_data_{system_data}
{
  parseHeader();
  parseSystemIndicators();
  parseMechanicalUnitGroups();
  parseRAPIDTasks();
  createStandardizedJoints(prefix);
}

RobotControllerDescription SystemDataParser::description()
{
  return description_;
}

/***********************************************************
 * Auxiliary methods (parsing)
 */

void SystemDataParser::parseHeader()
{
  auto header{description_.mutable_header()};

  header->set_ip_address(system_data_.ip_address);
  header->set_rws_port_number(system_data_.port_number);
  header->set_system_name(system_data_.system.system_name);
  header->set_system_type(system_data_.system.system_type);

  parseHeaderRobotWareVersion();

  for(const auto& option : system_data_.system.system_options)
  {
    header->add_options(option);
  }
}

void SystemDataParser::parseHeaderRobotWareVersion()
{
  auto version{description_.mutable_header()->mutable_robot_ware_version()};

  // The RobotWare version name format should be "<major>.<minor>.<patch>.<build> <optional description>".
  version->set_name(system_data_.system.robot_ware_version);

  // Look only for the "<major>.<minor>.<patch>." parts.
  std::regex regex{"([0-9]+\\.)"};
  auto begin{std::sregex_iterator(version->name().begin(), version->name().end(), regex)};

  // Parse the numbers.
  if(std::distance(begin, std::sregex_iterator()) == 3)
  {
    unsigned int value{0};
    unsigned int counter{0};

    for(auto it{begin}; it != std::sregex_iterator(); ++it)
    {
      auto match{it->str()};
      auto pos{match.find('.')};

      if(pos != std::string::npos)
      {
        std::stringstream ss{match.substr(0, pos)};

        ss >> value;
        if(ss.fail())
        {
          throw std::runtime_error{"Failed to parse RobotWare version string"};
        }

        switch(counter++)
        {
          case 0:
            version->set_major_number(value);
          break;

          case 1:
            version->set_minor_number(value);
          break;

          case 2:
            version->set_patch_number(value);
          break;

          default:
            // Do nothing.
          break;
        }
      }
    }
  }
  else
  {
    throw std::runtime_error{"Failed to parse RobotWare version string"};
  }
}

void SystemDataParser::parseSystemIndicators()
{
  auto system_indicators{description_.mutable_system_indicators()};

  for(const auto& option : system_data_.system.system_options)
  {
    if(option == "IRB 14000-0.5/0.5")
    {
      system_indicators->mutable_robots()->set_irb14000(true);
    }
    else if(option == "Leadthrough")
    {
      system_indicators->mutable_options()->set_leadthrough(true);
    }
    else if(option == "689-1 Externally Guided Motion (EGM)")
    {
      system_indicators->mutable_options()->set_egm(true);
    }
    else if(option == "604-1 MultiMove Coordinated" || option == "604-2 MultiMove Independent")
    {
      system_indicators->mutable_options()->set_multimove(true);
    }
    else if(option == "StateMachine")
    {
      system_indicators->mutable_addins()->set_state_machine_1_0(true);
    }
    else if(option == "StateMachine Core")
    {
      system_indicators->mutable_addins()->set_state_machine_1_1(true);
    }
    else if(option == "SmartGripper")
    {
      system_indicators->mutable_addins()->set_smart_gripper(true);
    }
  }
}

void SystemDataParser::parseMechanicalUnitGroups()
{
  const auto& cfg_mugs{system_data_.configurations.mechanical_unit_groups};
  const auto& cfg_mus{system_data_.configurations.mechanical_units};

  //--------------------------------------------------------
  // Parse the system data
  //
  // Note: Mechanical unit groups are only used in MultiMove
  //       systems. However, here it will also be used as a
  //       convenient container for non-MultiMove systems.
  //--------------------------------------------------------
  if(!description_.system_indicators().options().multimove())
  {
    // Create a "synthetic" mechanical unit group.
    auto mug{description_.add_mechanical_units_groups()};

    mug->set_name("");

    for(const auto& cfg_mu : cfg_mus)
    {
      if(!cfg_mu.use_robot.empty())
      {
        auto robot{findAndParseMechanicalUnit(cfg_mu.use_robot)};

        if(robot.type() == MechanicalUnit_Type_TCP_ROBOT)
        {
          mug->mutable_robot()->CopyFrom(robot);
        }
        else
        {
          mug->add_mechanical_units()->CopyFrom(robot);
        }
      }

      for(const auto& name : cfg_mu.use_singles)
      {
        if(!name.empty())
        {
          mug->add_mechanical_units()->CopyFrom(findAndParseMechanicalUnit(name));
        }
      }
    }
  }
  else
  {
    for(const auto& cfg_mug : cfg_mugs)
    {
      auto mug{description_.add_mechanical_units_groups()};

      mug->set_name(cfg_mug.name);

      if(!cfg_mug.robot.empty())
      {
        mug->mutable_robot()->CopyFrom(findAndParseMechanicalUnit(cfg_mug.robot));
      }

      for(const auto& name : cfg_mug.mechanical_units)
      {
        if(!name.empty())
        {
          mug->add_mechanical_units()->CopyFrom(findAndParseMechanicalUnit(name));
        }
      }
    }
  }

  sortMechanicalUnits();
}

void SystemDataParser::parseRAPIDTasks()
{
  for(const auto& task_data : system_data_.rapid_tasks)
  {
    auto rapid_task{description_.add_rapid_tasks()};

    rapid_task->set_name(task_data.name);
    rapid_task->set_is_motion_task(task_data.is_motion_task);
    rapid_task->set_is_active(task_data.is_active);

    switch(task_data.execution_state)
    {
      case rws::RWSInterface::RAPIDTaskExecutionState::READY:
        rapid_task->set_execution_state(RAPIDTask_ExecutionState_READY);
      break;

      case rws::RWSInterface::RAPIDTaskExecutionState::STOPPED:
        rapid_task->set_execution_state(RAPIDTask_ExecutionState_STOPPED);
      break;

      case rws::RWSInterface::RAPIDTaskExecutionState::STARTED:
        rapid_task->set_execution_state(RAPIDTask_ExecutionState_STARTED);
      break;

      case rws::RWSInterface::RAPIDTaskExecutionState::UNINITIALIZED:
        rapid_task->set_execution_state(RAPIDTask_ExecutionState_UNINITIALIZED);
      break;

      default:
        rapid_task->set_execution_state(RAPIDTask_ExecutionState_UNKNOWN);
    }

    for(const auto& module_data : task_data.modules)
    {
      auto module{rapid_task->add_modules()};
      module->set_name(module_data.name);
      module->set_type(module_data.type);
    }
  }
}

/***********************************************************
 * Auxiliary methods (finding)
 */

MechanicalUnit SystemDataParser::findAndParseMechanicalUnit(const std::string& name)
{
  const auto& cfg_mus{system_data_.configurations.mechanical_units};
  const auto& mus_extra{system_data_.mechanical_units_extra};

  auto it_cfg{std::find_if(cfg_mus.begin(), cfg_mus.end(), [&](const auto& x){return x.name == name;})};
  auto it_extra{mus_extra.find(name)};

  MechanicalUnit mu{};

  if(it_cfg != cfg_mus.end() && it_extra != mus_extra.end())
  {
    //------------------------------------------------------
    // Configuration information
    //------------------------------------------------------
    mu.set_name(it_cfg->name);

    if(!it_cfg->use_robot.empty())
    {
      mu.mutable_robot()->CopyFrom(findAndParseRobot(it_cfg->use_robot));
    }

    for(const auto& name : it_cfg->use_singles)
    {
      if(!name.empty())
      {
        mu.add_singles()->CopyFrom(findAndParseSingle(name));
      }
    }

    //------------------------------------------------------
    // Complementary static information
    //------------------------------------------------------
    switch(it_extra->second.static_info.type)
    {
      case rws::RWSInterface::MechanicalUnitType::NONE:
        mu.set_type(MechanicalUnit_Type_NONE);
      break;

      case rws::RWSInterface::MechanicalUnitType::TCP_ROBOT:
        mu.set_type(MechanicalUnit_Type_TCP_ROBOT);
      break;

      case rws::RWSInterface::MechanicalUnitType::ROBOT:
        mu.set_type(MechanicalUnit_Type_ROBOT);
      break;

      case rws::RWSInterface::MechanicalUnitType::SINGLE:
        mu.set_type(MechanicalUnit_Type_SINGLE);
      break;

      default:
        mu.set_type(MechanicalUnit_Type_UNDEFINED);
    }

    mu.set_task_name(it_extra->second.static_info.task_name);
    mu.set_axes(it_extra->second.static_info.axes);
    mu.set_axes_total(it_extra->second.static_info.axes_total);
    mu.set_is_integrated_unit(it_extra->second.static_info.is_integrated_unit);
    mu.set_has_integrated_unit(it_extra->second.static_info.has_integrated_unit);

    if(mu.is_integrated_unit() == Constants::NO_INTEGRATED_UNIT)
    {
      ++standalone_mechanical_unit_count_;
    }

    //------------------------------------------------------
    // Complementary dynamic information
    //------------------------------------------------------
    mu.set_status(it_extra->second.dynamic_info.status);

    switch (it_extra->second.dynamic_info.mode)
    {
      case rws::RWSInterface::MechanicalUnitMode::ACTIVATED:
        mu.set_mode(MechanicalUnit_Mode_ACTIVATED);
      break;

      default:
        mu.set_mode(MechanicalUnit_Mode_DEACTIVATED);
    }
  }

  return mu;
}

Robot SystemDataParser::findAndParseRobot(const std::string& name)
{
  const auto& cfg_robots{system_data_.configurations.robots};
  auto it{std::find_if(cfg_robots.begin(), cfg_robots.end(), [&](const auto& x){return x.name == name;})};

  Robot robot{};

  if(it != cfg_robots.end())
  {
    robot.set_name(it->name);
    robot.set_type(it->use_robot_type);

    for(const auto& name : it->use_joints)
    {
      if(!name.empty())
      {
        robot.add_joints()->CopyFrom(findAndParseJoint(name));
      }
    }

    robot.mutable_base_frame()->mutable_position()->set_x(it->base_frame.pos.x.value);
    robot.mutable_base_frame()->mutable_position()->set_y(it->base_frame.pos.y.value);
    robot.mutable_base_frame()->mutable_position()->set_z(it->base_frame.pos.z.value);
    robot.mutable_base_frame()->mutable_rotation()->set_q1(it->base_frame.rot.q1.value);
    robot.mutable_base_frame()->mutable_rotation()->set_q2(it->base_frame.rot.q2.value);
    robot.mutable_base_frame()->mutable_rotation()->set_q3(it->base_frame.rot.q3.value);
    robot.mutable_base_frame()->mutable_rotation()->set_q4(it->base_frame.rot.q4.value);
    robot.set_base_frame_moved_by(it->base_frame_moved_by);
  }

  return robot;
}

Single SystemDataParser::findAndParseSingle(const std::string& name)
{
  const auto& cfg_singles{system_data_.configurations.singles};
  auto it{std::find_if(cfg_singles.begin(), cfg_singles.end(), [&](const auto& x){return x.name == name;})};

  Single single{};

  if(it != cfg_singles.end())
  {
    single.set_name(it->name);
    single.set_type(it->use_single_type);

    if(!it->use_joint.empty())
    {
      single.mutable_joint()->CopyFrom(findAndParseJoint(it->use_joint));
    }

    single.mutable_base_frame()->mutable_position()->set_x(it->base_frame.pos.x.value);
    single.mutable_base_frame()->mutable_position()->set_y(it->base_frame.pos.y.value);
    single.mutable_base_frame()->mutable_position()->set_z(it->base_frame.pos.z.value);
    single.mutable_base_frame()->mutable_rotation()->set_q1(it->base_frame.rot.q1.value);
    single.mutable_base_frame()->mutable_rotation()->set_q2(it->base_frame.rot.q2.value);
    single.mutable_base_frame()->mutable_rotation()->set_q3(it->base_frame.rot.q3.value);
    single.mutable_base_frame()->mutable_rotation()->set_q4(it->base_frame.rot.q4.value);
    single.set_base_frame_moved_by(it->base_frame_coordinated);
  }

  return single;
}

Joint SystemDataParser::findAndParseJoint(const std::string& name)
{
  const auto& cfg_joints{system_data_.configurations.joints};
  auto it{std::find_if(cfg_joints.begin(), cfg_joints.end(), [&](const auto& x){return x.name == name;})};

  Joint joint{};

  if(it != cfg_joints.end())
  {
    joint.set_name(it->name);
    joint.set_logical_axis(it->logical_axis);
    joint.set_kinematic_axis_number(it->kinematic_axis_number);

    if(!it->use_arm.empty())
    {
      joint.mutable_arm()->CopyFrom(findAndParseArm(it->use_arm));
    }

    if(!it->use_transmission.empty())
    {
      joint.mutable_transmission()->CopyFrom(findAndParseTransmission(it->use_transmission));
    }
  }

  return joint;
}

Arm SystemDataParser::findAndParseArm(const std::string& name)
{
  const auto& cfg_arms{system_data_.configurations.arms};
  auto it{std::find_if(cfg_arms.begin(), cfg_arms.end(), [&](const auto& x){return x.name == name;})};

  Arm arm{};

  if(it != cfg_arms.end())
  {
    arm.set_name(it->name);
    arm.set_lower_joint_bound(it->lower_joint_bound);
    arm.set_upper_joint_bound(it->upper_joint_bound);
  }

  return arm;
}

Transmission SystemDataParser::findAndParseTransmission(const std::string& name)
{
  const auto& cfg_transmissions{system_data_.configurations.transmissions};
  auto it{std::find_if(cfg_transmissions.begin(), cfg_transmissions.end(),[&](const auto& x){return x.name == name;})};

  Transmission transmission{};

  if(it != cfg_transmissions.end())
  {
    transmission.set_name(it->name);
    transmission.set_rotating_move(it->rotating_move);
  }

  return transmission;
}

/***********************************************************
 * Auxiliary methods
 */

void SystemDataParser::sortMechanicalUnits()
{
  for(int i{0}; i < description_.mechanical_units_groups_size(); ++i)
  {
    auto mugs{description_.mutable_mechanical_units_groups(i)->mutable_mechanical_units()};
    std::sort(mugs->begin(), mugs->end(), [](MechanicalUnit mu1, MechanicalUnit mu2)
    {
      int mu1_min_logical_axis{99};
      int mu2_min_logical_axis{99};

      //----------------------------------------------------
      // Unit 1
      //----------------------------------------------------
      if(mu1.has_robot())
      {
        for(const auto& joint : mu1.robot().joints())
        {
          if(mu1_min_logical_axis > joint.logical_axis())
          {
            mu1_min_logical_axis = joint.logical_axis();
          }
        }
      }

      for(const auto& single : mu1.singles())
      {
        if(mu1_min_logical_axis > single.joint().logical_axis())
        {
          mu1_min_logical_axis = single.joint().logical_axis();
        }
      }

      //----------------------------------------------------
      // Unit 2
      //----------------------------------------------------
      if(mu2.has_robot())
      {
        for(const auto& joint : mu2.robot().joints())
        {
          if(mu2_min_logical_axis > joint.logical_axis())
          {
            mu2_min_logical_axis = joint.logical_axis();
          }
        }
      }

      for(const auto& single : mu2.singles())
      {
        if(mu2_min_logical_axis > single.joint().logical_axis())
        {
          mu2_min_logical_axis = single.joint().logical_axis();
        }
      }

      return mu1_min_logical_axis < mu2_min_logical_axis;
    });
  }
}

void SystemDataParser::extractJoints(const MechanicalUnit& unit, std::vector<Joint>& joints)
{
  if(unit.has_robot())
  {
    for(const auto& joint : unit.robot().joints())
    {
      joints.push_back(joint);
    }
  }

  for(const auto& single : unit.singles())
  {
    joints.push_back(single.joint());
  }

  if(unit.has_integrated_unit() != Constants::NO_INTEGRATED_UNIT)
  {
    extractJoints(findAndParseMechanicalUnit(unit.has_integrated_unit()), joints);
  }
}

std::vector<Joint> SystemDataParser::extractAndSortJoints(const MechanicalUnit& unit)
{
  std::vector<Joint> joints{};

  extractJoints(unit, joints);

  std::sort(joints.begin(), joints.end(), [](Joint j1, Joint j2)
  {
    if(j1.kinematic_axis_number() != -1 && j2.kinematic_axis_number() != -1)
    {
      return j1.kinematic_axis_number() < j2.kinematic_axis_number();
    }
    else
    {
      return j1.logical_axis() < j2.logical_axis();
    }
  });

  return joints;
}

void SystemDataParser::createStandardizedJoints(const std::string& prefix)
{
  //--------------------------------------------------------
  // Support lambdas
  //--------------------------------------------------------
  auto standardizedJointName{[&](std::string mech_unit, const int joint_number)
  {
    std::stringstream ss{};

    if(!prefix.empty())
    {
      ss << prefix << "_";
    }

    if(standalone_mechanical_unit_count_ > 1)
    {
      // Remove known (potential) underscore separator characters and transform into lower case letters.
      mech_unit.erase(std::remove(mech_unit.begin(), mech_unit.end(), '_'), mech_unit.end());
      std::transform(mech_unit.begin(), mech_unit.end(), mech_unit.begin(), [](auto c){return std::tolower(c);});

      ss << mech_unit << "_";
    }

    ss << "joint_" << joint_number;

    return ss.str();
  }};

  auto addStandardizedJoints{[&](MechanicalUnit* p_mu)
  {
    if(!p_mu)
    {
      return;
    }

    auto sorted_joints{extractAndSortJoints(*p_mu)};

    for(size_t i{0}; i < sorted_joints.size(); ++i)
    {
      auto standardized_joint{p_mu->add_standardized_joints()};
      standardized_joint->set_standardized_name(standardizedJointName(p_mu->name(), i + 1));
      standardized_joint->set_original_name(sorted_joints[i].name());
      standardized_joint->set_rotating_move(sorted_joints[i].transmission().rotating_move());
      standardized_joint->set_lower_joint_bound(sorted_joints[i].arm().lower_joint_bound());
      standardized_joint->set_upper_joint_bound(sorted_joints[i].arm().upper_joint_bound());
    }
  }};

  //--------------------------------------------------------
  // Loop over all known mechanical unit groups
  //--------------------------------------------------------
  for(int i{0}; i < description_.mechanical_units_groups_size(); ++i)
  {
    auto p_mug{description_.mutable_mechanical_units_groups(i)};

    if(p_mug->has_robot() && p_mug->robot().is_integrated_unit() == Constants::NO_INTEGRATED_UNIT)
    {
      addStandardizedJoints(p_mug->mutable_robot());
    }

    for(int j{0}; j < p_mug->mechanical_units_size(); ++j)
    {
      if(p_mug->mechanical_units(j).is_integrated_unit() == Constants::NO_INTEGRATED_UNIT)
      {
        addStandardizedJoints(p_mug->mutable_mechanical_units(j));
      }
    }
  }
}

}
}
