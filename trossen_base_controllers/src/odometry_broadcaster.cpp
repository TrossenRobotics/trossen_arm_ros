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

#include "trossen_base_controllers/odometry_broadcaster.hpp"

#include <cmath>

namespace trossen_base_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

CallbackReturn OdometryBroadcaster::on_init()
{
  auto_declare<std::string>("position_x_interface", "base/position_x");
  auto_declare<std::string>("position_y_interface", "base/position_y");
  auto_declare<std::string>("position_yaw_interface", "base/position_yaw");
  auto_declare<std::string>("velocity_x_interface", "base/velocity_x");
  auto_declare<std::string>("velocity_y_interface", "base/velocity_y");
  auto_declare<std::string>("velocity_yaw_interface", "base/velocity_yaw");

  auto_declare<std::string>("odom_frame_id", "odom");
  auto_declare<std::string>("base_frame_id", "base_footprint");

  auto_declare<bool>("enable_odom_tf", true);

  // Covariance diagonals (x, y, z, roll, pitch, yaw)
  auto_declare<std::vector<double>>(
    "pose_covariance_diagonal", std::vector<double>(COVARIANCE_DIM_, 0.0));
  auto_declare<std::vector<double>>(
    "twist_covariance_diagonal", std::vector<double>(COVARIANCE_DIM_, 0.0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = get_node();

  // Same order as the INDEX_* constants
  state_interface_names_ = {
    node->get_parameter("position_x_interface").as_string(),
    node->get_parameter("position_y_interface").as_string(),
    node->get_parameter("position_yaw_interface").as_string(),
    node->get_parameter("velocity_x_interface").as_string(),
    node->get_parameter("velocity_y_interface").as_string(),
    node->get_parameter("velocity_yaw_interface").as_string()};
  for (const std::string & name : state_interface_names_) {
    if (name.empty()) {
      RCLCPP_ERROR(node->get_logger(), "State interface parameters must not be empty");
      return CallbackReturn::ERROR;
    }
  }

  odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = node->get_parameter("base_frame_id").as_string();
  enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();

  const std::vector<double> pose_covariance =
    node->get_parameter("pose_covariance_diagonal").as_double_array();
  const std::vector<double> twist_covariance =
    node->get_parameter("twist_covariance_diagonal").as_double_array();
  if (pose_covariance.size() != COVARIANCE_DIM_ || twist_covariance.size() != COVARIANCE_DIM_) {
    RCLCPP_ERROR(node->get_logger(), "Covariance diagonals must have %zu values", COVARIANCE_DIM_);
    return CallbackReturn::ERROR;
  }

  // Odometry message: frames and covariance never change, so fill them once
  odom_msg_ = nav_msgs::msg::Odometry();
  odom_msg_.header.frame_id = odom_frame_id_;
  odom_msg_.child_frame_id = base_frame_id_;
  for (size_t i = 0; i < COVARIANCE_DIM_; ++i) {
    odom_msg_.pose.covariance[i * COVARIANCE_DIM_ + i] = pose_covariance[i];
    odom_msg_.twist.covariance[i * COVARIANCE_DIM_ + i] = twist_covariance[i];
  }
  odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  realtime_odom_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_publisher_);

  // Transform message: a single odom -> base transform with fixed frames
  if (enable_odom_tf_) {
    tf_msg_ = tf2_msgs::msg::TFMessage();
    tf_msg_.transforms.resize(1);
    tf_msg_.transforms[0].header.frame_id = odom_frame_id_;
    tf_msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    realtime_tf_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_publisher_);
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration OdometryBroadcaster::command_interface_configuration() const
{
  // Read-only: claims no command interfaces
  return {interface_configuration_type::NONE, {}};
}

InterfaceConfiguration OdometryBroadcaster::state_interface_configuration() const
{
  return {interface_configuration_type::INDIVIDUAL, state_interface_names_};
}

return_type OdometryBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  // Read all states but skip this cycle if any value is unavailable
  double values[NUM_STATES_];
  for (size_t i = 0; i < NUM_STATES_; ++i) {
    values[i] = state_interfaces_[i].get_value();
    if (std::isnan(values[i])) {
      return return_type::OK;
    }
  }

  // Planar base: orientation is a rotation of yaw about z
  const double yaw = values[INDEX_POSITION_YAW_];
  const double qz = std::sin(yaw / 2.0);
  const double qw = std::cos(yaw / 2.0);

  // Odometry: pose in odom frame, twist in base frame
  odom_msg_.header.stamp = time;
  odom_msg_.pose.pose.position.x = values[INDEX_POSITION_X_];
  odom_msg_.pose.pose.position.y = values[INDEX_POSITION_Y_];
  odom_msg_.pose.pose.orientation.z = qz;
  odom_msg_.pose.pose.orientation.w = qw;
  odom_msg_.twist.twist.linear.x = values[INDEX_VELOCITY_X_];
  odom_msg_.twist.twist.linear.y = values[INDEX_VELOCITY_Y_];
  odom_msg_.twist.twist.angular.z = values[INDEX_VELOCITY_YAW_];
  if (realtime_odom_publisher_->trylock()) {
    realtime_odom_publisher_->msg_ = odom_msg_;
    realtime_odom_publisher_->unlockAndPublish();
  }

  // Transform odom -> base
  if (enable_odom_tf_) {
    geometry_msgs::msg::TransformStamped & transform = tf_msg_.transforms[0];
    transform.header.stamp = time;
    transform.transform.translation.x = values[INDEX_POSITION_X_];
    transform.transform.translation.y = values[INDEX_POSITION_Y_];
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
    if (realtime_tf_publisher_->trylock()) {
      realtime_tf_publisher_->msg_ = tf_msg_;
      realtime_tf_publisher_->unlockAndPublish();
    }
  }

  return return_type::OK;
}

}  // namespace trossen_base_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trossen_base_controllers::OdometryBroadcaster,
  controller_interface::ControllerInterface)
