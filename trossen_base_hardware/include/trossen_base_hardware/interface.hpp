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

#include <array>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace trossen_base_hardware
{


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using hardware_interface::CommandInterface;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::InterfaceInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

// Every interface is defined in the URDF <ros2_control> block and accessed by index,
// so the order of joints, sensors, gpios and their interfaces in the URDF matters.
class TrossenBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrossenBaseHardwareInterface)

  ~TrossenBaseHardwareInterface() override;

  CallbackReturn on_init(const HardwareInfo & info) override;

  std::vector<StateInterface> export_state_interfaces() override;

  std::vector<CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & duration) override;

  return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

protected:
  std::unordered_map<std::string, double> state_values_;
  std::unordered_map<std::string, double> command_values_;

  void set_state(const std::string & interface_name, double value);
  double get_command(const std::string & interface_name) const;
  void set_command(const std::string & interface_name, double value);

  // Logger
  static rclcpp::Logger get_logger()
  {
    return rclcpp::get_logger("trossen_base_hardware");
  }

  void start_lidar_node();
  void stop_lidar_node();
  rclcpp::Node::SharedPtr lidar_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr lidar_executor_;
  std::thread lidar_executor_thread_;

  // Mock mode read(): loops commands back into states without the driver
  return_type read_mock(const rclcpp::Duration & period);

  // Returns the command value, or 0 if no controller has written it yet (NaN)
  double get_command_or_zero(const std::string & interface_name) const;

  void lidar_timer_callback();
  trossen_base::PointCloud mock_lidar_pointcloud() const;

  // Lidar pointcloud publisher
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  int64_t last_lidar_timestamp_ms_{0};

  // Index of each joint in the URDF <ros2_control> block (order matters)
  static constexpr size_t INDEX_LIFT_ = 0;
  static constexpr size_t INDEX_FRONT_LEFT_PIVOT_ = 1;
  static constexpr size_t INDEX_FRONT_LEFT_DRIVE_ = 2;
  static constexpr size_t INDEX_FRONT_RIGHT_PIVOT_ = 3;
  static constexpr size_t INDEX_FRONT_RIGHT_DRIVE_ = 4;
  static constexpr size_t INDEX_BACK_PIVOT_ = 5;
  static constexpr size_t INDEX_BACK_DRIVE_ = 6;
  static constexpr size_t NUM_JOINTS_ = 7;

  // Index of each sensor in the URDF <ros2_control> block (order matters)
  static constexpr size_t INDEX_IMU_ = 0;
  static constexpr size_t INDEX_BATTERY_ = 1;
  static constexpr size_t NUM_SENSORS_ = 2;

  // Index of each gpio in the URDF <ros2_control> block (order matters)
  static constexpr size_t INDEX_BASE_ = 0;
  static constexpr size_t NUM_GPIOS_ = 1;

  // Number of interfaces accessed by index (order matters):
  // IMU states: orientation x/y/z/w, angular_velocity x/y/z, linear_acceleration x/y/z
  static constexpr size_t NUM_IMU_STATES_ = 10;
  // Battery states: voltage, current, percentage, temperature, power_supply_status
  static constexpr size_t NUM_BATTERY_STATES_ = 5;
  // Base commands: velocity x/y/yaw
  static constexpr size_t NUM_BASE_COMMANDS_ = 3;
  // Base states: position x/y/yaw, velocity x/y/yaw
  static constexpr size_t NUM_BASE_STATES_ = 6;

  // The driver for the robot
  std::unique_ptr<trossen_base::TrossenBase> base_driver_{nullptr};

  // Drive wheel radius (m), converts drive velocity m/s -> rad/s
  double wheel_radius_ = 0.0;

  // Mock mode: no driver, commands are looped back into states
  bool mock_ = false;

  // Topic for the lidar PointCloud2, set by the optional "lidar_topic" hardware parameter
  std::string lidar_topic_ = "lidar/points";

  // Mock base pose x, y, yaw in odom frame (m, rad), integrated from velocity commands
  std::array<double, 3> mock_base_position_{};

  // Mock lift position (m), integrated from velocity commands
  double mock_lift_position_ = 0.0;

  // Mock lift position limits (m), set by the optional "lift_min_position" and
  // "lift_max_position" hardware parameters (unlimited if not set)
  double lift_min_position_ = -std::numeric_limits<double>::infinity();
  double lift_max_position_ = std::numeric_limits<double>::infinity();
};

}  // namespace trossen_base_hardware

#endif  // TROSSEN_BASE_HARDWARE__INTERFACE_HPP_
