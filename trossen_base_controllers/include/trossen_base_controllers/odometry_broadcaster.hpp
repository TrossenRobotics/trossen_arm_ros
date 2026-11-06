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

#ifndef TROSSEN_BASE_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_
#define TROSSEN_BASE_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace trossen_base_controllers
{

// Publishes base odometry from the hardware's state interfaces, which already hold the driver's
// fused odometry: position x/y/yaw in the odom frame, velocity x/y/yaw in the base frame.
class OdometryBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Index of each state interface in state_interfaces_, same order as
  // state_interface_configuration()
  static constexpr size_t INDEX_POSITION_X_ = 0;
  static constexpr size_t INDEX_POSITION_Y_ = 1;
  static constexpr size_t INDEX_POSITION_YAW_ = 2;
  static constexpr size_t INDEX_VELOCITY_X_ = 3;
  static constexpr size_t INDEX_VELOCITY_Y_ = 4;
  static constexpr size_t INDEX_VELOCITY_YAW_ = 5;
  static constexpr size_t NUM_STATES_ = 6;

  // Covariance matrices are COVARIANCE_DIM_ x COVARIANCE_DIM_ (x, y, z, roll, pitch, yaw),
  // stored row-major in nav_msgs/Odometry
  static constexpr size_t COVARIANCE_DIM_ = 6;

  // Full state interface names, in the order above
  std::vector<std::string> state_interface_names_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool enable_odom_tf_ = true;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  realtime_odom_publisher_;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
  realtime_tf_publisher_;

  nav_msgs::msg::Odometry odom_msg_;
  tf2_msgs::msg::TFMessage tf_msg_;
};

}  // namespace trossen_base_controllers

#endif  // TROSSEN_BASE_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_
