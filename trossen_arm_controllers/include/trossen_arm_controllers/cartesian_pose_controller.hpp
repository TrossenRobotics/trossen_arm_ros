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

#ifndef TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_
#define TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trossen_arm_hardware/interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using controller_interface::ControllerInterface;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using trossen_arm_hardware::HW_IF_CARTESIAN_POSE;
using geometry_msgs::msg::PoseStamped;

namespace trossen_arm_controllers
{

/**
 * @brief Controller for commanding Cartesian poses to the robot end-effector.
 */
class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  CartesianPoseController() = default;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  InterfaceConfiguration command_interface_configuration() const override;
  InterfaceConfiguration state_interface_configuration() const override;
  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * @brief Callback for receiving target pose commands.
   * @param msg The target pose message in the base frame.
   */
  void target_pose_callback(const PoseStamped::SharedPtr msg);

  /**
   * @brief Convert quaternion to axis-angle representation.
   *
   * Converts a quaternion to axis-angle representation where the result is a 3D vector whose
   * direction is the rotation axis and whose magnitude is the rotation angle in radians.
   *
   * @param qx Quaternion x component.
   * @param qy Quaternion y component.
   * @param qz Quaternion z component.
   * @param qw Quaternion w component (scalar part).
   * @return Array of 3 doubles [rx, ry, rz] representing axis-angle in radians.
   */
  std::array<double, 3> quaternion_to_axis_angle(
    double qx,
    double qy,
    double qz,
    double qw) const;

  /// @brief Names of the GPIO interfaces to command (must be size 6)
  std::vector<std::string> gpio_names_;

  /// @brief Subscriber to target pose commands
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_target_pose_;

  /// @brief Realtime buffer for passing commands from callback to control loop
  realtime_tools::RealtimeBuffer<std::array<double, 6>> rt_command_buffer_;
};

}  // namespace trossen_arm_controllers

#endif  // TROSSEN_ARM_CONTROLLERS__CARTESIAN_POSE_CONTROLLER_HPP_
