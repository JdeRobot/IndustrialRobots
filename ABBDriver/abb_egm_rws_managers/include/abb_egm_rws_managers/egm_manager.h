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

#ifndef ABB_EGM_RWS_MANAGERS_EGM_MANAGER_H
#define ABB_EGM_RWS_MANAGERS_EGM_MANAGER_H

#include <memory>
#include <thread>

#include <abb_libegm/egm_controller_interface.h>

#include "utilities.h"

namespace abb
{
namespace robot
{

/**
 * \brief Manager for handling Externally Guided Motion (EGM) communication with an ABB robot controller.
 */
class EGMManager
{
public:
  /**
   * \brief Configurations for an EGM channel.
   */
  struct ChannelConfiguration
  {
    /**
     * \brief Creates an EGM channel configuration.
     *
     * \param port_number used by the EGM channel.
     * \param mech_unit_group assumed to be connected to the EGM channel.
     */
    ChannelConfiguration(const unsigned short port_number, const MechanicalUnitGroup& mech_unit_group)
    :
    port_number{port_number},
    mech_unit_group{mech_unit_group}
    {}

    /**
     * \brief Port number used by the EGM channel.
     */
    unsigned short port_number;

    /**
     * \brief Mechanical unit group assumed to be connected to the EGM channel.
     */
    MechanicalUnitGroup mech_unit_group;
  };

  /**
   * \brief Creates a manager for handling EGM communication with the robot controller's EGM clients.
   *
   * \param channel_configurations specifying the EGM channels to setup and manage.
   */
  EGMManager(const std::vector<ChannelConfiguration>& channel_configurations);

  /**
   * \brief Destroys the manager and stops internal communication threads.
   */
  ~EGMManager();

  /**
   * \brief Waits for a message on any of the configured EGM channels.
   *
   * \param timeout_ms for the amount of time [ms] to wait before timing out (zero means waiting indefinitely).
   *
   * \return false if a timeout occurred.
   */
  bool waitForMessage(const unsigned int timeout_ms);

  /**
   * \brief Reads states from all configured EGM channels.
   *
   * \param motion_data for storing the states.
   *
   * \return true if any new states were read.
   */
  bool read(MotionData& motion_data);

  /**
   * \brief Writes commands to all configured EGM channels.
   *
   * \param motion_data containing the commands.
   */
  void write(const MotionData& motion_data);

private:
  /**
   * \brief Channel for handling EGM communication with one of the robot controller's EGM clients.
   */
  class Channel
  {
  public:
    /**
     * \brief Creates a channel for handling EGM communication with one of the robot controller's EGM clients.
     *
     * \param configuration with specifications for the channel.
     * \param io_service that manages asynchronous UDP communication events.
     * \param p_new_message_cv condition variable for new message notifications.
     *
     * \throw std::runtime_error if failed to create an EGM interface for the channel.
     */
    Channel(const ChannelConfiguration& configuration,
            boost::asio::io_service& io_service,
            boost::shared_ptr<boost::condition_variable> p_new_message_cv);

    /**
     * \brief Reads states from the channel.
     *
     * \param group for storing the states.
     *
     * \return true if any new states were read.
     */
    bool read(MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Writes commands to the channel.
     *
     * \param group containing the commands.
     */
    void write(const MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Gets the name of the mechanical unit group that is assumed to be connected to the channel.
     *
     * \return const std::string& reference with the name.
     */
    const std::string& getMechanicalUnitGroupName() const;

  private:
    /**
     * \brief Counts the number of active TCP robot joints.
     *
     * \param group containing the joints.
     *
     * \return int with the number of active joints.
     */
    int countActiveTCPRobotJoints(const MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Counts the number of active external joints.
     *
     * \param group containing the joints.
     *
     * \return int with the number of active joints.
     */
    int countActiveExternalJoints(const MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Updates the TCP robot joint states.
     *
     * \param group for storing the joint states.
     *
     * \return true if any new joint states were read.
     *
     * \throw std::runtime_error if there is a mismatch between number of assumed active robot joints and feedback.
     */
    bool updateTCPRobotJointStates(MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Updates the external joint states.
     *
     * \param group for storing the joint states.
     *
     * \return true if any new joint states were read.
     *
     * \throw std::runtime_error if there is a mismatch between number of assumed active external joints and feedback.
     */
    bool updateExternalJointStates(MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Prepares output commands.
     */
    void prepareOutputs();

    /**
     * \brief Updates the TCP robot joint commands.
     *
     * \param group containing the joint commands.
     */
    void updateTCPRobotJointCommands(const MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Updates the external joint commands.
     *
     * \param group containing the joint commands.
     */
    void updateExternalJointCommands(const MotionData::MechanicalUnitGroup& group);

    /**
     * \brief Validates a joint command.
     *
     * \param joint containing the command.
     *
     * \throw std::invalid_argument if any of the commanded values are NaN or Inf.
     * \throw std::out_of_range if any of the commanded values are out-of-bounds.
     */
    void validateJointCommand(const MotionData::Joint& joint);

    /**
     * \brief The channel's configurations.
     */
    ChannelConfiguration configuration_;

    /**
     * \brief The EGM communication interface.
     */
    std::unique_ptr<egm::EGMControllerInterface> p_interface_;

    /**
     * \brief Inputs read from the interface.
     */
    egm::wrapper::Input input_;

    /**
     * \brief Previous header read from the interface.
     */
    egm::wrapper::Header previous_header_;

    /**
     * \brief Previous status read from the interface.
     */
    egm::wrapper::Status previous_status_;

    /**
     * \brief Outputs to write to interface.
     */
    egm::wrapper::Output output_;

    /**
     * \brief Indicator for if the channel is active (i.e. messages are being received or not).
     */
    bool channel_is_active_;

    /**
     * \brief Number of missed messages.
     */
    unsigned int missed_messages_;
  };

  /**
   * \brief Manages asynchronous UDP communication events.
   */
  boost::asio::io_service io_service_;

  /**
   * \brief Threads for processing asynchronous UDP communication events.
   */
  std::vector<std::thread> threads_;

  /**
   * \brief EGM channels to manage.
   */
  std::vector<Channel> channels_;

  /**
   * \brief Mutex protection for the new message condition variable.
   */
  boost::mutex new_message_mutex_;

  /**
   * \brief Condition variable for new message notifications.
   */
  boost::shared_ptr<boost::condition_variable> p_new_message_cv_;
};

}
}

#endif
