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

#ifndef ABB_EGM_RWS_MANAGERS_RWS_MANAGER_H
#define ABB_EGM_RWS_MANAGERS_RWS_MANAGER_H

#include <mutex>
#include <functional>

#include <abb_librws/rws_state_machine_interface.h>

#include "utilities.h"

namespace abb
{
namespace robot
{

/**
 * \brief Manager for handling Robot Web Service (RWS) communication with an ABB robot controller.
 */
class RWSManager
{
public:
  using ServiceFunctor = std::function<void(rws::RWSStateMachineInterface& interface)>;

  /**
   * \brief Creates a manager for handling communication with the robot controller's RWS server.
   *
   * \param ip_address to the RWS server.
   * \param port_number used by the RWS server.
   * \param username for the RWS authentication process.
   * \param password for the RWS authentication process.
   */
  RWSManager(const std::string& ip_address,
             const unsigned short port_number,
             const std::string& username,
             const std::string& password);

  /**
   * \brief Collects key data, about the robot controller's active system, and parses it into a structured description.
   *
   * \param prefix for standardized joint names (i.e. arbitrary prefix for identifying a specific robot controller).
   *
   * \return RobotControllerDescription of the robot controller.
   *
   * \throw std::runtime_error if a handleable error happened (e.g. communication timed out).
   */
  RobotControllerDescription collectAndParseSystemData(const std::string& prefix);

 /**
   * \brief Collects runtime data, about the robot controller's active system, and updates the runtime data containers.
   *
   * This includes general system states and motion information of the system's mechanical units.
   *
   * \param system_state_data container for storing the general system states.
   * \param motion_data container for storing the motion states.
   *
   * \throw std::runtime_error if a handleable error happened (e.g. communication timed out).
   * \throw std::logic_error if an unhandleable error happened (e.g. switching to another robot controller system).
   */
  void collectAndUpdateRuntimeData(SystemStateData& system_state_data, MotionData& motion_data);

  /**
   * \brief Checks if the low priority RWS interface is ready to be used.
   *
   * \return bool true if the interface is ready.
   */
  bool isInterfaceReady();

  /**
   * \brief Runs the provided service with the low priority RWS interface.
   *
   * Notes:
   * - Only runs the service if the interface is free.
   * - Intended for lower priority services.
   *
   * \param service to run.
   *
   * \return bool true if the service was run.
   */
  bool runService(ServiceFunctor const& service);

  /**
   * \brief Runs the provided service with the high priority RWS interface.
   *
   * Notes:
   * - Waits until the interface is free before running the service.
   * - Intended for higher priority services.
   *
   * \param service to run.
   */
  void runPriorityService(ServiceFunctor const& service);

  /**
   * \brief Creates a debug text of the latest connection attempt.
   *
   * I.e. a summary of collected key data, about the robot controller's active system,
   * and the structured description parsed from the key data.
   *
   * \return std::string containing the debug text.
   */
  std::string debugText() const;

private:
  /**
   * \brief Mutex for protecting the low priority RWS interface.
   */
  std::mutex interface_mutex_;

  /**
   * \brief Mutex for protecting the high priority RWS interface.
   */
  std::mutex priority_interface_mutex_;

  /**
   * \brief RWS communication interface, intended for lower priority requests.
   */
  rws::RWSStateMachineInterface interface_;

  /**
   * \brief RWS communication interface, intended for higher priority requests.
   */
  rws::RWSStateMachineInterface priority_interface_;

  /**
   * \brief Key data about the robot controller's active system (in raw, unstructured, format).
   */
  SystemData system_data_;

  /**
   * \brief Structured description of the connected robot controller.
   */
  RobotControllerDescription description_;

  /**
   * \brief The robot controller's runtime system state.
   */
  SystemStateData system_state_data_;
};

}
}

#endif
