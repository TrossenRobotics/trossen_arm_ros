// Copyright 2026 Trossen Robotics
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

#ifndef TROSSEN_BASE_HARDWARE__INTERFACE_HPP_
#define TROSSEN_BASE_HARDWARE__INTERFACE_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "trossen_base/trossen_base.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trossen_base_hardware
{


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using hardware_interface::InterfaceInfo;
using hardware_interface::InterfaceDescription;
using hardware_interface::HardwareInfo;
using hardware_interface::HardwareComponentInterfaceParams;
using hardware_interface::return_type;
using hardware_interface::StateInterface;


class TrossenBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrossenBaseHardwareInterface)

  CallbackReturn on_init(const HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override;
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

protected:
  // The driver for the robot
  std::unique_ptr<trossen_base::TrossenBase> base_driver_{nullptr};

};

}  // namespace trossen_base_hardware

#endif  // TROSSEN_BASE_HARDWARE__INTERFACE_HPP_
