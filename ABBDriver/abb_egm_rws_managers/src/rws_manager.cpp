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

#include <stdexcept>

#include "abb_egm_rws_managers/rws_manager.h"
#include "abb_egm_rws_managers/system_data_parser.h"

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Class definitions: RWSManager
 */

/***********************************************************
 * Primary methods
 */

RWSManager::RWSManager(const std::string& ip_address,
                       const unsigned short port_number,
                       const std::string& username,
                       const std::string& password)
:
interface_{ip_address, port_number, username, password},
priority_interface_{ip_address, port_number, username, password}
{
  // Set a higher timeout [microseconds] that is, for
  // example, needed when turning motors on for multiple
  // mechanical units.
  interface_.setHTTPTimeout(1e6);
  priority_interface_.setHTTPTimeout(1e6);

  system_data_.ip_address = ip_address;
  system_data_.port_number = port_number;
}

/***********************************************************
 * Primary methods (for the lower priority RWS interface)
 */

RobotControllerDescription RWSManager::collectAndParseSystemData(const std::string& prefix)
{
  //--------------------------------------------------------
  // Collect the system data
  //--------------------------------------------------------
  {
    std::lock_guard<std::mutex> guard{interface_mutex_};

    //------------------------
    // General system info
    //------------------------
    system_data_.system = interface_.getSystemInfo();

    if(system_data_.system.robot_ware_version.empty() ||
       system_data_.system.system_name.empty() ||
       system_data_.system.system_type.empty() ||
       system_data_.system.system_options.empty())
    {
      throw std::runtime_error{"Failed to collect general system info"};
    }

    //------------------------
    // Configurations
    //------------------------
    system_data_.configurations.mechanical_unit_groups = interface_.getCFGMechanicalUnitGroups();
    system_data_.configurations.mechanical_units = interface_.getCFGMechanicalUnits();

    if(system_data_.configurations.mechanical_units.empty())
    {
      throw std::runtime_error{"Failed to collect mechanical unit configurations"};
    }

    system_data_.configurations.robots = interface_.getCFGRobots();
    system_data_.configurations.singles = interface_.getCFGSingles();
    system_data_.configurations.joints = interface_.getCFGJoints();

    if(system_data_.configurations.joints.empty())
    {
      throw std::runtime_error{"Failed to collect joint configurations"};
    }

    system_data_.configurations.arms = interface_.getCFGArms();

    if(system_data_.configurations.arms.empty())
    {
      throw std::runtime_error{"Failed to collect arm configurations"};
    }

    system_data_.configurations.transmissions = interface_.getCFGTransmission();

    if(system_data_.configurations.transmissions.empty())
    {
      throw std::runtime_error{"Failed to collect transmission configurations"};
    }

    //------------------------
    // Complementary info for
    // mechanical units
    //------------------------
    for(const auto& mech_unit : system_data_.configurations.mechanical_units)
    {
      std::pair<std::string, SystemData::MechanicalUnit> pair{};

      pair.first = mech_unit.name;

      if(interface_.getMechanicalUnitStaticInfo(pair.first, pair.second.static_info) &&
         interface_.getMechanicalUnitDynamicInfo(pair.first, pair.second.dynamic_info))
      {
        system_data_.mechanical_units_extra.insert(pair);
      }
      else
      {
        throw std::runtime_error{"Failed to collect complementary mechanical unit info"};
      }
    }

    //------------------------
    // RAPID tasks
    //------------------------
    auto rapid_tasks{interface_.getRAPIDTasks()};

    if(rapid_tasks.empty())
    {
      throw std::runtime_error{"Failed to collect RAPID task info"};
    }

    for(const auto& rapid_task : rapid_tasks)
    {
      system_data_.rapid_tasks.emplace_back(rapid_task, interface_.getRAPIDModulesInfo(rapid_task.name));
    }
  }

  //--------------------------------------------------------
  // Parse the collected (raw) system data into a structured
  // description of the connected robot controller
  //--------------------------------------------------------
  description_ = SystemDataParser{system_data_, prefix}.description();

  return description_;
}

void RWSManager::collectAndUpdateRuntimeData(SystemStateData& system_state_data, MotionData& motion_data)
{
  std::lock_guard<std::mutex> guard{interface_mutex_};

  system_state_data.reset();

  //--------------------------------------------------------
  // System state data
  //--------------------------------------------------------
  // General data
  //--------------------------
  system_state_data.system_name = interface_.getSystemInfo().system_name;
  if(system_state_data.system_name.empty())
  {
    throw std::runtime_error{"Failed to connect to robot controller"};
  }
  if(system_state_data.system_name != description_.header().system_name())
  {
    throw std::logic_error{"System name mismatch"};
  }

  system_state_data.motors_on = interface_.isMotorsOn();
  if(system_state_data.motors_on.isUnknown())
  {
    throw std::runtime_error{"Motor on/off state unknown"};
  }

  system_state_data.auto_mode = interface_.isAutoMode();
  if(system_state_data.auto_mode.isUnknown())
  {
    throw std::runtime_error{"Controller auto/manual state unknown"};
  }

  system_state_data.rapid_running = interface_.isRAPIDRunning();
  if(system_state_data.rapid_running.isUnknown())
  {
    throw std::runtime_error{"RAPID running/stopped state unknown"};
  }

  //--------------------------
  // Mechanical units
  //--------------------------
  rws::RWSInterface::MechanicalUnitStaticInfo static_info{};
  rws::RWSInterface::MechanicalUnitDynamicInfo dynamic_info{};
  rws::JointTarget joint_target{};

  for(const auto& group : description_.mechanical_units_groups())
  {
    if(group.has_robot() && group.robot().is_integrated_unit() == Constants::NO_INTEGRATED_UNIT)
    {
      if(!interface_.getMechanicalUnitDynamicInfo(group.robot().name(), dynamic_info))
      {
        throw std::runtime_error{"Robot info missing"};
      }
      if(!interface_.getMechanicalUnitJointTarget(group.robot().name(), &joint_target))
      {
        throw std::runtime_error{"Robot joint info missing"};
      }

      // Reorder joint values for special cases (e.g. for IRB14000).
      if(group.robot().axes_total() == 7)
      {
        double temp{joint_target.extax.eax_a.value};
        joint_target.extax.eax_a.value = joint_target.robax.rax_6.value;
        joint_target.robax.rax_6.value = joint_target.robax.rax_5.value;
        joint_target.robax.rax_5.value = joint_target.robax.rax_4.value;
        joint_target.robax.rax_4.value = joint_target.robax.rax_3.value;
        joint_target.robax.rax_3.value = temp;
      }

      std::pair<std::string, SystemStateData::MechanicalUnit> pair{};
      pair.first = group.robot().name();
      pair.second.active = dynamic_info.mode == rws::RWSInterface::ACTIVATED;
      pair.second.joint_target = joint_target;
      system_state_data.mechanical_units.insert(pair);
    }

    for(const auto& unit : group.mechanical_units())
    {
      if(unit.is_integrated_unit() == Constants::NO_INTEGRATED_UNIT)
      {
        if(!interface_.getMechanicalUnitStaticInfo(unit.name(), static_info) ||
           !interface_.getMechanicalUnitDynamicInfo(unit.name(), dynamic_info))
        {
          throw std::runtime_error{"Mechanical unit info missing"};
        }
        if(!interface_.getMechanicalUnitJointTarget(unit.name(), &joint_target))
        {
          throw std::runtime_error{"Mechanical unit joint info missing"};
        }
        if(unit.task_name() != static_info.task_name)
        {
          throw std::logic_error{"Mechanical unit RAPID task name mismatch"};
        }

        std::pair<std::string, SystemStateData::MechanicalUnit> pair{};
        pair.first = unit.name();
        pair.second.active = dynamic_info.mode == rws::RWSInterface::ACTIVATED;
        pair.second.joint_target = joint_target;
        system_state_data.mechanical_units.insert(pair);
      }
    }
  }

  //--------------------------
  // RAPID tasks
  //--------------------------
  system_state_data.rapid_tasks = interface_.getRAPIDTasks();

  if(system_state_data.rapid_tasks.empty())
  {
    throw std::runtime_error{"RAPID task info missing"};
  }

  //--------------------------------------------------------
  // Motion data
  //--------------------------------------------------------
  // Loop over all known mechanical unit groups.
  for(auto& group : motion_data.groups)
  {
    for(auto& unit : group.units)
    {
      const auto it{system_state_data.mechanical_units.find(unit.name)};
      if(it != system_state_data.mechanical_units.end())
      {
        unit.active = it->second.active;

        for(size_t i{0}; i < unit.joints.size(); ++i)
        {
          auto& joint{unit.joints[i]};

          auto conversion_factor{(joint.rotational ? Constants::DEG_TO_RAD : Constants::MM_TO_M)};

          // Map RAPID joint target to joint state position.
          switch(i)
          {
            case 0:
              joint.state.position = it->second.joint_target.robax.rax_1.value*conversion_factor;
            break;

            case 1:
              joint.state.position = it->second.joint_target.robax.rax_2.value*conversion_factor;
            break;

            case 2:
              joint.state.position = it->second.joint_target.robax.rax_3.value*conversion_factor;
            break;

            case 3:
              joint.state.position = it->second.joint_target.robax.rax_4.value*conversion_factor;
            break;

            case 4:
              joint.state.position = it->second.joint_target.robax.rax_5.value*conversion_factor;
            break;

            case 5:
              joint.state.position = it->second.joint_target.robax.rax_6.value*conversion_factor;
            break;

            case 6:
              joint.state.position = it->second.joint_target.extax.eax_a.value*conversion_factor;
            break;

            case 7:
              joint.state.position = it->second.joint_target.extax.eax_b.value*conversion_factor;
            break;

            case 8:
              joint.state.position = it->second.joint_target.extax.eax_c.value*conversion_factor;
            break;

            case 9:
              joint.state.position = it->second.joint_target.extax.eax_d.value*conversion_factor;
            break;

            case 10:
              joint.state.position = it->second.joint_target.extax.eax_e.value*conversion_factor;
            break;

            case 11:
              joint.state.position = it->second.joint_target.extax.eax_f.value*conversion_factor;
            break;

            default:
              // Do nothing.
            break;
          }
        }
      }
      else
      {
        throw std::logic_error{"Mechanical unit could not be found"};
      }
    }
  }

  //--------------------------------------------------------
  // RobotWare StateMachine Add-In states
  //--------------------------------------------------------
  auto indicators = description_.system_indicators();

  if(indicators.addins().state_machine_1_0() ||
     indicators.addins().state_machine_1_1())
  {
    // Loop over all known RAPID motion tasks.
    for(const auto& task : system_state_data.rapid_tasks)
    {
      if(task.is_motion_task)
      {
        SystemStateData::SateMachine state_machine{};

        // Store the related RAPID task's name.
        state_machine.rapid_task = task.name;

        // Get the current state machine state.
        interface_.getRAPIDSymbolData(task.name, "TRobMain", "current_state", &state_machine.sm_state);

        // Get the current EGM action.
        if(indicators.options().egm())
        {
          interface_.getRAPIDSymbolData(task.name, "TRobEGM", "current_action", &state_machine.egm_action);

          // Map special case from v1.0 to v1.1 values
          // (i.e. map "ACTION_STOP" from "0" to "3").
          if(indicators.addins().state_machine_1_0())
          {
            if(static_cast<int>(state_machine.egm_action.value) == 0)
            {
              state_machine.egm_action.value = 3;
            }
          }
        }
        else
        {
          state_machine.egm_action = rws::RWSStateMachineInterface::EGMActions::EGM_ACTION_UNKNOWN;
        }
        system_state_data.state_machines.push_back(state_machine);
      }
    }
  }
}

bool RWSManager::isInterfaceReady()
{
  if(interface_mutex_.try_lock())
  {
    interface_mutex_.unlock();
    return true;
  }

  return false;
}

bool RWSManager::runService(ServiceFunctor const& service)
{
  if(interface_mutex_.try_lock())
  {
    service(interface_);
    interface_mutex_.unlock();
    return true;
  }

  return false;
}

/***********************************************************
 * Primary methods (for the higher priority RWS interface)
 */

void RWSManager::runPriorityService(ServiceFunctor const& service)
{
  std::lock_guard<std::mutex> guard{priority_interface_mutex_};
  service(priority_interface_);
}

/***********************************************************
 * Auxiliary methods
 */

std::string RWSManager::debugText() const
{
  //--------------------------------------------------------
  // Support lambdas
  //--------------------------------------------------------
  auto levelOne{[](){ return "  |- "; }};
  auto levelTwo{[](std::string connector){ return "  " + connector + "  |- "; }};
  auto connectorString{[](size_t i, size_t size){ return (i == size - 1 ? " " : "|"); }};
  auto boolToString{[](bool b){ return (b ? "True" : "False"); }};

  auto stringVectorWithoutEmpty{[](std::vector<std::string> sv)
  {
    auto it{std::remove_if(sv.begin(), sv.end(),[](std::string s){return s.empty();})};
    sv.erase(it, sv.end());
    return sv;
  }};

  auto stringVectorToString{[](std::vector<std::string> sv)
  {
    std::stringstream ss{};

    for(size_t i{0}; i < sv.size(); ++i)
    {
      ss << sv[i] << (i == sv.size() - 1 ? "" : ", ");
    }

    return ss.str();
  }};

  //--------------------------------------------------------
  // Create the debug text
  //--------------------------------------------------------
  std::stringstream ss{};

  ss << std::setprecision(2) << std::fixed;

  const auto& header{description_.header()};
  ss << "Robot controller at '" << header.ip_address() << ":" << header.rws_port_number() << "'\n";

  ss << "\n"
     << "============================================================\n"
     << "= System indicators (parsed data)\n"
     << "============================================================\n";

  const auto& indicators{description_.system_indicators()};

  ss << "# Robots:\n"
     << levelOne() << "IRB14000 (a.k.a. YuMi): " << boolToString(indicators.robots().irb14000()) << "\n"
     << "\n"
     << "# RobotWare Options:\n"
     << levelOne() << "EGM: " << boolToString(indicators.options().egm()) << "\n"
     << levelOne() << "Leadthrough: " << boolToString(indicators.options().leadthrough()) << "\n"
     << levelOne() << "MultiMove: " << boolToString(indicators.options().multimove()) << "\n"
     << "\n"
     << "# RobotWare Add-Ins:\n"
     << levelOne() << "SmartGripper: " << boolToString(indicators.addins().smart_gripper()) << "\n"
     << levelOne() << "StateMachine 1.0: " << boolToString(indicators.addins().state_machine_1_0()) << "\n"
     << levelOne() << "StateMachine 1.1: " << boolToString(indicators.addins().state_machine_1_1()) << "\n";

  ss << "\n"
     << "============================================================\n"
     << "= System configurations (raw data)\n"
     << "============================================================\n";

  //--------------------------------------------------------
  // Arm instances
  //--------------------------------------------------------
  ss << "# Arms:\n";
  for(const auto& arm : system_data_.configurations.arms)
  {
    ss << levelOne() << arm.name << " [" << arm.lower_joint_bound << ", " << arm.upper_joint_bound << "]\n";
  }

  //--------------------------------------------------------
  // Transmission instances
  //--------------------------------------------------------
  ss << "\n# Transmissions:\n";
  for(const auto& transmission : system_data_.configurations.transmissions)
  {
    ss << levelOne() << transmission.name << " [rotational: " << boolToString(transmission.rotating_move) << "]\n";
  }

  //--------------------------------------------------------
  // Joint instances
  //--------------------------------------------------------
  ss << "\n# Joints:\n";
  for(const auto& joint : system_data_.configurations.joints)
  {
    ss << levelOne() << joint.name
       << " [logical axis: " << joint.logical_axis
       << ", kinematic axis: " << joint.kinematic_axis_number
       << ", using arm: " << joint.use_arm
       << ", using transmission: " << joint.use_transmission << "]\n";
  }

  //--------------------------------------------------------
  // Single instances
  //--------------------------------------------------------
  const auto& singles{system_data_.configurations.singles};
  if(!singles.empty())
  {
    ss << "\n# Singles:\n";
    for(size_t i{0}; i < singles.size(); ++i)
    {
      auto connector{connectorString(i, singles.size())};

      ss << levelOne() << singles[i].name << "\n"
         << levelTwo(connector) << "Type: " << singles[i].use_single_type << "\n"
         << levelTwo(connector) << "Joint: " << singles[i].use_joint << "\n"
         << levelTwo(connector) << "Base frame: " << singles[i].base_frame.constructString() << "\n";

      if(!singles[i].base_frame_coordinated.empty())
      {
        ss << levelTwo(connector) << "Base coordinated: " << singles[i].base_frame_coordinated << "\n";
      }
    }
  }

  //--------------------------------------------------------
  // Robot instances
  //--------------------------------------------------------
  const auto& robots{system_data_.configurations.robots};
  if(!robots.empty())
  {
    ss << "\n# Robots:\n";
    for(size_t i{0}; i < robots.size(); ++i)
    {
      auto connector{connectorString(i, robots.size())};

      ss << levelOne() << robots[i].name << "\n"
         << levelTwo(connector) << "Type: " << robots[i].use_robot_type << "\n";

      auto temp{stringVectorWithoutEmpty(robots[i].use_joints)};
      if(!temp.empty())
      {
        ss << levelTwo(connector) << "Joint" << (temp.size() == 1 ? ": " : "s: [")
                                  << stringVectorToString(temp)
                                  << (temp.size() == 1 ? "\n" : "]\n");
      }

      ss << levelTwo(connector) << "Base frame: " << robots[i].base_frame.constructString() << "\n";

      if(!robots[i].base_frame_moved_by.empty())
      {
        ss << levelTwo(connector) << "Base frame moved by: " << robots[i].base_frame_moved_by << "\n";
      }
    }
  }

  //--------------------------------------------------------
  // Mechanical unit instances
  //--------------------------------------------------------
  ss << "\n# Mechanical Units:\n";
  auto& mech_units{system_data_.configurations.mechanical_units};
  for(size_t i{0}; i < mech_units.size(); ++i)
  {
    auto connector{connectorString(i, mech_units.size())};

    ss << levelOne() << mech_units[i].name << "\n";

    if(!mech_units[i].use_robot.empty())
    {
      ss << levelTwo(connector) << "Robot: " << mech_units[i].use_robot << "\n";
    }

    auto temp{stringVectorWithoutEmpty(mech_units[i].use_singles)};
    if(!temp.empty())
    {
      ss << levelTwo(connector) << "Single" << (temp.size() == 1 ? ": " : "s: [")
                                << stringVectorToString(temp)
                                << (temp.size() == 1 ? "\n" : "]\n");
    }
  }

  //--------------------------------------------------------
  // Mechanical unit group instances
  //--------------------------------------------------------
  if(indicators.options().multimove())
  {
    ss << "\n# Mechanical Unit Groups:\n";
    auto& mech_unit_groups{system_data_.configurations.mechanical_unit_groups};

    for(size_t i{0}; i < mech_unit_groups.size(); ++i)
    {
      auto connector{connectorString(i, mech_unit_groups.size())};

      ss << levelOne() << mech_unit_groups[i].name << "\n";

      if(!mech_unit_groups[i].robot.empty())
      {
        ss << levelTwo(connector) << "Robot: " << mech_unit_groups[i].robot << "\n";
      }

      auto temp{stringVectorWithoutEmpty(mech_unit_groups[i].mechanical_units)};
      if(!temp.empty())
      {
        ss << levelTwo(connector) << "Mechanical unit" << (temp.size() == 1 ? ": " : "s: [")
                                  << stringVectorToString(temp)
                                  << (temp.size() == 1 ? "\n" : "]\n");
      }
    }
  }

  ss << "\n"
     << "============================================================\n"
     << "= System configurations (parsed data)\n"
     << "============================================================\n";

  //--------------------------------------------------------
  // Parsed mechanical unit groups
  //--------------------------------------------------------
  auto mugs_size{description_.mechanical_units_groups_size()};
  for(int i{0}; i < mugs_size; ++i)
  {
    ss << "= Mechanical Unit Group [" << i + 1 << "/" << mugs_size << "]\n"
       << "=============================\n"
       << description_.mechanical_units_groups(i).DebugString()
       << (i == mugs_size - 1 ? "" : "\n=============================\n");
  }

  ss << "============================================================";

  return ss.str();
}

}
}
