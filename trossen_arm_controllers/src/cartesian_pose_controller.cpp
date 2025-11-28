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

#include "trossen_arm_controllers/cartesian_pose_controller.hpp"

#include <cmath>
#include <limits>

namespace trossen_arm_controllers
{

CallbackReturn
CartesianPoseController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("gpio_names", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during 'gpio_names' parameter declaration: %s",
      e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Read gpio_names parameter
  try {
    gpio_names_ = get_node()->get_parameter("gpio_names").as_string_array();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to retrieve parameter 'gpio_names': %s",
      e.what());
    return CallbackReturn::ERROR;
  }

  // Validate parameter
  if (gpio_names_.size() != 6) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'gpio_names' must contain exactly 6 names (pos_x/y/z, rot_x/y/z), got %zu",
      gpio_names_.size());
    return CallbackReturn::ERROR;
  }

  // Create subscription to target pose
  sub_target_pose_ = get_node()->create_subscription<PoseStamped>(
    "~/target_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianPoseController::target_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured CartesianPoseController with %zu GPIO interfaces",
    gpio_names_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the correct number of command interfaces
  if (command_interfaces_.size() != 6) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 6 command interfaces, got %zu",
      command_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  // Initialize command buffer with NaN (hardware interface will handle first update)
  std::array<double, 6> initial_command;
  initial_command.fill(std::numeric_limits<double>::quiet_NaN());
  rt_command_buffer_.writeFromNonRT(initial_command);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Activated CartesianPoseController - ready to receive pose commands on topic '%s'",
    sub_target_pose_->get_topic_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated CartesianPoseController");
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const
{
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & gpio_name : gpio_names_) {
    config.names.push_back(gpio_name + "/" + HW_IF_CARTESIAN_POSE);
  }

  return config;
}

InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const
{
  // This controller does not use state interfaces
  return InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

return_type
CartesianPoseController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Read command from realtime buffer
  auto command_ptr = rt_command_buffer_.readFromRT();

  if (command_ptr) {
    const auto & command = *command_ptr;

    // Check if command contains valid data (not all NaN)
    bool has_valid_data = false;
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isnan(command[i])) {
        has_valid_data = true;
        break;
      }
    }

    if (has_valid_data) {
      // Write command to all 6 interfaces
      for (size_t i = 0; i < 6; ++i) {
        command_interfaces_[i].set_value(command[i]);
      }
    }
  }

  return return_type::OK;
}

void CartesianPoseController::target_pose_callback(const PoseStamped::SharedPtr msg)
{
  // Convert quaternion to axis-angle
  auto axis_angle = quaternion_to_axis_angle(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);

  // Pack command: [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
  std::array<double, 6> command = {
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z,
    axis_angle[0],
    axis_angle[1],
    axis_angle[2]
  };

  // Write to realtime buffer
  rt_command_buffer_.writeFromNonRT(command);
}

std::array<double, 3> CartesianPoseController::quaternion_to_axis_angle(
  double qx,
  double qy,
  double qz,
  double qw) const
{
  // Normalize quaternion to handle potential floating-point errors
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-10) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received near-zero quaternion, returning identity rotation");
    return {0.0, 0.0, 0.0};
  }

  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  // Ensure w is positive to get the shortest rotation
  if (qw < 0.0) {
    qx = -qx;
    qy = -qy;
    qz = -qz;
    qw = -qw;
  }

  // Compute rotation angle
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(qw)));

  // Handle identity rotation (angle ~= 0)
  if (angle < 1e-6) {
    return {0.0, 0.0, 0.0};
  }

  // Compute axis-angle representation
  // For quaternion q = [sin(θ/2) * axis, cos(θ/2)]
  // axis-angle = angle * axis = angle * [qx, qy, qz] / sin(θ/2)
  // Simplify: angle / sin(angle/2) = 2 * acos(qw) / sqrt(1 - qw²)
  const double sin_half_angle = std::sqrt(1.0 - qw * qw);

  if (sin_half_angle < 1e-6) {
    // Handle 180-degree rotation case (qw ≈ 0)
    // Find the largest component to avoid numerical issues
    const double scale = angle;
    return {qx * scale, qy * scale, qz * scale};
  }

  // Standard case
  const double scale = angle / sin_half_angle;
  return {qx * scale, qy * scale, qz * scale};
}

}  // namespace trossen_arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trossen_arm_controllers::CartesianPoseController,
  controller_interface::ControllerInterface)
