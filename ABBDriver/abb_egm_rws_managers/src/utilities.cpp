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
#include <sstream>
#include <stdexcept>

#include "abb_egm_rws_managers/utilities.h"

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Struct definitions: Constants
 */

constexpr double Constants::RAD_TO_DEG;
constexpr double Constants::DEG_TO_RAD;
constexpr double Constants::MM_TO_M;
constexpr double Constants::M_TO_MM;
constexpr char Constants::NO_INTEGRATED_UNIT[];

/***********************************************************************************************************************
 * Utility function definitions
 */

MechanicalUnitGroup findMechanicalUnitGroup(const std::string& name, const RobotControllerDescription& description)
{
  MechanicalUnitGroup mug{};

  auto const& mugs{description.mechanical_units_groups()};
  auto it{std::find_if(mugs.begin(), mugs.end(), [&](const auto& x){return x.name() == name;})};

  if(it == mugs.end())
  {
    throw std::runtime_error{"Failed to find mechanical unit group '" + name + "' in the robot controller description"};
  }

  mug.CopyFrom(*it);

  return mug;
}

void initializeMotionData(MotionData& motion_data, const RobotControllerDescription& description)
{
  const auto& rw_version{description.header().robot_ware_version()};

  //--------------------------------------------------------
  // Support lambda
  //--------------------------------------------------------
  auto createMotionUnit{[&](const MechanicalUnit& unit)
  {
    MotionData::MechanicalUnit motion_unit{};

    motion_unit.name = unit.name();
    motion_unit.type = unit.type();
    motion_unit.active = unit.mode() == MechanicalUnit_Mode_ACTIVATED;
    motion_unit.supported_by_egm = false;

    // Set indicator for if the unit is supported by EGM or not.
    if(unit.type() == MechanicalUnit_Type_TCP_ROBOT)
    {
      // TCP robots:
      // - Six axes robots are supported by EGM.
      // - Seven axes robots are supported by EGM since RobotWare 6.09.
      if(unit.axes_total() == 6)
      {
        motion_unit.supported_by_egm = true;
      }
      else if(unit.axes_total() == 7)
      {
        if(rw_version.major_number() > 6 ||
          (rw_version.major_number() == 6 && rw_version.minor_number() >= 9))
        {
          motion_unit.supported_by_egm = true;
        }
      }
    }
    else if(unit.type() == MechanicalUnit_Type_ROBOT || unit.type() == MechanicalUnit_Type_SINGLE)
    {
      // External axes are supported by EGM since RobotWare 6.07.
      if(rw_version.major_number() > 6 ||
         (rw_version.major_number() == 6 && rw_version.minor_number() >= 7))
      {
        motion_unit.supported_by_egm = true;
      }
    }

    // Add joint information.
    for(const auto& standardized_joint : unit.standardized_joints())
    {
      MotionData::Joint motion_joint{};
      motion_joint.name = standardized_joint.standardized_name();
      motion_joint.rotational = standardized_joint.rotating_move();
      motion_joint.lower_limit = standardized_joint.lower_joint_bound();
      motion_joint.upper_limit = standardized_joint.upper_joint_bound();
      motion_joint.state.position = 0.0;
      motion_joint.state.velocity = 0.0;
      motion_joint.state.effort = 0.0;
      motion_joint.command.position = 0.0;
      motion_joint.command.velocity = 0.0;
      motion_unit.joints.push_back(motion_joint);
    }

    return motion_unit;
  }};

  //--------------------------------------------------------
  // Loop over all known mechanical unit groups
  //--------------------------------------------------------
  for(const auto& group : description.mechanical_units_groups())
  {
    MotionData::MechanicalUnitGroup motion_group{};

    motion_group.name = group.name();
    motion_group.egm_channel_data.was_activated_or_deactivated = false;
    motion_group.egm_channel_data.is_active = false;

    if(group.has_robot() && !group.robot().standardized_joints().empty())
    {
      if(group.robot().axes_total() != group.robot().standardized_joints_size())
      {
        throw std::runtime_error{"Number of robot axes are not equal to expected number of joints"};
      }

      motion_group.units.push_back(createMotionUnit(group.robot()));
    }

    for(const auto& unit : group.mechanical_units())
    {
      if(!unit.standardized_joints().empty())
      {
        if(unit.axes_total() != unit.standardized_joints_size())
        {
          throw std::runtime_error{"Number of external axes are not equal to expected number of joints"};
        }

        motion_group.units.push_back(createMotionUnit(unit));
      }
    }

    motion_data.groups.push_back(motion_group);
  }
}

std::string summaryText(const RobotControllerDescription& description)
{
  //--------------------------------------------------------
  // Support lambdas
  //--------------------------------------------------------
  auto levelOne{[](){ return "  |- "; }};
  auto levelTwo{[](){ return "     |- "; }};

  //--------------------------------------------------------
  // Create the summary text
  //--------------------------------------------------------
  std::stringstream ss{};

  ss << "============================================================\n"
     << "= Summary of robot controller at '" << description.header().ip_address()
     << ":" << description.header().rws_port_number() << "'\n"
     << "============================================================\n";

  ss << "# General Information:\n"
     << levelOne() << "RobotWare version: " << description.header().robot_ware_version().name() << "\n"
     << levelOne() << "System name: " << description.header().system_name() << "\n"
     << levelOne() << "System type: " << description.header().system_type() << "\n"
     << levelOne() << "Options:\n";

  for(const auto& option : description.header().options())
  {
    ss << levelTwo() << option << "\n";
  }

  ss << "\n# Mechanical Units:\n";

  for(const auto& group : description.mechanical_units_groups())
  {
    if(group.has_robot())
    {
      auto active{group.robot().mode() == MechanicalUnit_Mode_ACTIVATED};
      ss << levelOne() << "Unit: " << group.robot().name() << (active ? "\n" : " (currently deactivated)\n");
    }

    for(const auto& unit : group.mechanical_units())
    {
      auto active{unit.mode() == MechanicalUnit_Mode_ACTIVATED};
      ss << levelOne() << "Unit: " << unit.name() << (active ? "\n" : " (currently deactivated)\n");
    }
  }

  ss << "\n# Mechanical Unit Groups:\n";

  if(description.system_indicators().options().multimove())
  {
    for(const auto& group : description.mechanical_units_groups())
    {
      ss << levelOne() << "Group: " << group.name() << "\n";
    }
  }
  else
  {
      ss << levelOne() << "N/A (only for MultiMove systems)\n";
  }

  ss << "============================================================";

  return ss.str();
}

}
}
