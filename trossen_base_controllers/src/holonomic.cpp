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

#include "trossen_base_controllers/holonomic.hpp"

namespace trossen_base_controllers
{

CallbackReturn HolonomicController::on_init()
{
  auto_declare<std::string>("velocity_x_interface", "base/velocity_x");
  auto_declare<std::string>("velocity_y_interface", "base/velocity_y");
  auto_declare<std::string>("velocity_yaw_interface", "base/velocity_yaw");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HolonomicController::on_configure(const rclcpp_lifecycle::State&)
{

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = get_node();
  velocity_x_interface_ = node->get_parameter("velocity_x_interface").as_string();
  velocity_y_interface_ = node->get_parameter("velocity_y_interface").as_string();
  velocity_yaw_interface_ = node->get_parameter("velocity_yaw_interface").as_string();

  for (const std::string& name :
    {velocity_x_interface_, velocity_y_interface_, velocity_yaw_interface_}) {
    if (name.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Velocity interface parameters must not be empty");
      return CallbackReturn::ERROR;
    }
  }

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_.writeFromNonRT(*msg);
    });

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration HolonomicController::command_interface_configuration() const
{
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {velocity_x_interface_, velocity_y_interface_, velocity_yaw_interface_};
  return config;
}

InterfaceConfiguration HolonomicController::state_interface_configuration() const
{
  // This controller does not use state interfaces
  return {};
}

return_type HolonomicController::update( const rclcpp::Time&, const rclcpp::Duration&)
{

  const geometry_msgs::msg::Twist & cmd = *cmd_vel_buffer_.readFromRT();

  if (!command_interfaces_[0].set_value(cmd.linear.x) ||
      !command_interfaces_[1].set_value(cmd.linear.y) ||
      !command_interfaces_[2].set_value(cmd.angular.z))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set velocity command");
  }

  return return_type::OK;
}

}  // namespace trossen_base_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trossen_base_controllers::HolonomicController,
  controller_interface::ControllerInterface)
