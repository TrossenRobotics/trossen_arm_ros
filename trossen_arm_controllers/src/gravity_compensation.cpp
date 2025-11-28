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

#include "trossen_arm_controllers/gravity_compensation.hpp"

namespace trossen_arm_controllers
{

CallbackReturn
GravityCompensationController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize node parameters.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to retrieve parameter 'joints' needed by GravityCompensationController.");
    return CallbackReturn::ERROR;
  }
  if (joint_names_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "List of joints to command must not be empty");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + HW_IF_EXTERNAL_EFFORT);
  }

  return config;
}

InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  // This controller does not use state interfaces
  return InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

return_type
GravityCompensationController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Set all command interfaces to zero external effort
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return return_type::OK;
}

}  // namespace trossen_arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trossen_arm_controllers::GravityCompensationController,
  controller_interface::ControllerInterface)
