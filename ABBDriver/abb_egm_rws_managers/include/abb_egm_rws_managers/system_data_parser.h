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

#ifndef ABB_EGM_RWS_MANAGERS_SYSTEM_DATA_PARSER_H
#define ABB_EGM_RWS_MANAGERS_SYSTEM_DATA_PARSER_H

#include <string>
#include <vector>

#include "utilities.h"

namespace abb
{
namespace robot
{

/**
 * \brief ABB robot controller system data parser that outputs a description of the robot controller.
 */
class SystemDataParser
{
public:
  /**
   * \brief Creates a parser for constructing a robot controller description from system data.
   *
   * \param system_data of the robot controller's active system.
   * \param prefix for standardized joint names (i.e. arbitrary prefix for identifying a specific robot controller).
   */
  SystemDataParser(const SystemData& system_data, const std::string& prefix);

  /**
   * \brief Gets the parsed robot controller description.
   *
   * \return RobotControllerDescription of the robot controller.
   */
  RobotControllerDescription description();

private:
  /**
   * \brief Parses the system data's header.
   */
  void parseHeader();

  /**
   * \brief Parses the system data's RobotWare version name into subcomponents (i.e. major, minor and patch numbers).
   *
   * \throw std::runtime_error if failed to parse the version name.
   */
  void parseHeaderRobotWareVersion();

  /**
   * \brief Parses the system data into key indicators.
   *
   * Checks for the presence of certain RobotWare options and Add-Ins, as well as special robot cases.
   */
  void parseSystemIndicators();

  /**
   * \brief Parses the system data's mechanical unit groups.
   */
  void parseMechanicalUnitGroups();

  /**
   * \brief Parses the system data's RAPID tasks.
   */
  void parseRAPIDTasks();

  /**
   * \brief Finds and parses a mechanical unit instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return MechanicalUnit containing the parsed instance.
   */
  MechanicalUnit findAndParseMechanicalUnit(const std::string& name);

  /**
   * \brief Finds and parses a robot instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return Robot containing the parsed instance.
   */
  Robot findAndParseRobot(const std::string& name);

  /**
   * \brief Finds and parses a single instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return Single containing the parsed instance.
   */
  Single findAndParseSingle(const std::string& name);

  /**
   * \brief Finds and parses a joint instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return Joint containing the parsed instance.
   */
  Joint findAndParseJoint(const std::string& name);

  /**
   * \brief Finds and parses an arm instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return Arm containing the parsed instance.
   */
  Arm findAndParseArm(const std::string& name);

  /**
   * \brief Finds and parses a transmission instance in the system data's configurations.
   *
   * \param name of the instance.
   *
   * \return Transmission containing the parsed instance.
   */
  Transmission findAndParseTransmission(const std::string& name);

  /**
   * \brief Sorts all mechanical unit groups' mechanical units in ascending order.
   *
   * The sorting condition is based on the lowest logical axis number of the units' joints.
   */
  void sortMechanicalUnits();

  /**
   * \brief Extracts all joints from a mechanical unit.
   *
   * Including joints of any integrated mechanical units.
   *
   * \param unit to extract joints from.
   * \param joints for containing the extracted joints.
   */
  void extractJoints(const MechanicalUnit& unit, std::vector<Joint>& joints);

  /**
   * \brief Extracts and sorts all joints from a mechanical unit.
   *
   * Including joints of any integrated mechanical units.
   *
   * The sorting condition is based on:
   * - The lowest kinematic axis number (if available).
   * - The lowest logical axis number (if kinematic axis number is unavailable).
   *
   * \param std::vector<Joint> containing the extracted and sorted joints.
   */
  std::vector<Joint> extractAndSortJoints(const MechanicalUnit& unit);

  /**
   * \brief Creates standardized joints for all mechanical unit groups' mechanical units.
   *
   * \param prefix for the standardized joint names.
   */
  void createStandardizedJoints(const std::string& prefix);

  /**
   * \brief Count of the number of standalone mechanical units.
   *
   * I.e. mechanical units that are not integrated into another mechanical unit.
   */
  unsigned int standalone_mechanical_unit_count_;

  /**
   * \brief The system data used during the parsing.
   */
  SystemData system_data_;

  /**
   * \brief The resulting description from the parsed system data.
   */
  RobotControllerDescription description_;
};

}
}

#endif
