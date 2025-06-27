// Copyright 2025 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TROSSEN_ARM_HARDWARE__INTERFACE_HPP_
#define TROSSEN_ARM_HARDWARE__INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "libtrossen_arm/trossen_arm.hpp"

namespace trossen_arm_hardware
{

static const char DRIVER_IP_ADDRESS_DEFAULT[] = "192.168.1.2";

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using trossen_arm::TrossenArmDriver;

class TrossenArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrossenArmHardwareInterface)

  CallbackReturn on_init(const HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

protected:
  // The driver for the robot
  std::unique_ptr<TrossenArmDriver> arm_driver_{nullptr};

  trossen_arm::Model robot_model_;

  std::string driver_ip_address_{DRIVER_IP_ADDRESS_DEFAULT};

  // Joint positions in radians
  std::vector<double> joint_positions_;

  // Joint velocities in radians per second
  std::vector<double> joint_velocities_;

  // Joint efforts in Newton-meters
  std::vector<double> joint_efforts_;

  // Joint position commands in radians
  std::vector<double> joint_position_commands_;

  // Flag to indicate the first read/write update
  bool first_update_{true};

  const size_t COUNT_COMMAND_INTERFACES_ = 1;  // position
  const size_t INDEX_COMMAND_INTERFACE_POSITION_ = 0;
  const size_t COUNT_STATE_INTERFACES_ = 3;  // position, velocity, effort
  const size_t INDEX_STATE_INTERFACE_POSITION_ = 0;
  const size_t INDEX_STATE_INTERFACE_VELOCITY_ = 1;
  const size_t INDEX_STATE_INTERFACE_EFFORT_ = 2;

  // Logger
  static rclcpp::Logger get_logger()
  {
    return rclcpp::get_logger("trossen_arm_hardware");
  }
};

}  // namespace trossen_arm_hardware

#endif  // TROSSEN_ARM_HARDWARE__INTERFACE_HPP_
