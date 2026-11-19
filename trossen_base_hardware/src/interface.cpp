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

#include "trossen_base_hardware/interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

namespace trossen_base_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
TrossenBaseHardwareInterface::on_init(const HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(get_logger(), "Missing hardware parameter 'wheel_radius'");
    return CallbackReturn::ERROR;
  } catch (const std::invalid_argument &) {
    RCLCPP_FATAL(get_logger(), "Hardware parameter 'wheel_radius' is not a number");
    return CallbackReturn::ERROR;
  }

  // Must be positive, it is divided by in read()
  if (wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "Hardware parameter 'wheel_radius' must be > 0");
    return CallbackReturn::ERROR;
  }

  // Components are accessed by index in read()/write(), so the URDF must list exactly these:
  // joints: lift, fl_pivot, fl_drive, fr_pivot, fr_drive, b_pivot, b_drive
  // sensors: imu, battery
  // gpios: base
  if (info_.joints.size() != NUM_JOINTS_ || info_.sensors.size() != NUM_SENSORS_ ||
    info_.gpios.size() != NUM_GPIOS_)
  {
    RCLCPP_FATAL(get_logger(),
      "Expected %zu joints, %zu sensors, %zu gpios in <ros2_control>, got %zu, %zu, %zu",
      NUM_JOINTS_, NUM_SENSORS_, NUM_GPIOS_,
      info_.joints.size(), info_.sensors.size(), info_.gpios.size());
    return CallbackReturn::ERROR;
  }

  // IMU, battery and base interfaces are also accessed by index, so check their counts
  if (info_.sensors[INDEX_IMU_].state_interfaces.size() != NUM_IMU_STATES_ ||
    info_.sensors[INDEX_BATTERY_].state_interfaces.size() != NUM_BATTERY_STATES_ ||
    info_.gpios[INDEX_BASE_].command_interfaces.size() != NUM_BASE_COMMANDS_ ||
    info_.gpios[INDEX_BASE_].state_interfaces.size() != NUM_BASE_STATES_)
  {
    RCLCPP_FATAL(get_logger(), "IMU, battery or base has the wrong number of interfaces");
    return CallbackReturn::ERROR;
  }

  // Optional mock mode
  std::unordered_map<std::string, std::string>::const_iterator mock_param =
    info_.hardware_parameters.find("mock");
  mock_ = mock_param != info_.hardware_parameters.end() && mock_param->second == "true";

  // Lift position limits from the URDF <limit>, used to clamp the mock lift
  lift_min_position_ = -std::numeric_limits<double>::infinity();
  lift_max_position_ = std::numeric_limits<double>::infinity();
  std::unordered_map<std::string, joint_limits::JointLimits>::const_iterator lift_limits =
    info_.limits.find(info_.joints[INDEX_LIFT_].name);
  if (lift_limits != info_.limits.end() && lift_limits->second.has_position_limits) {
    lift_min_position_ = lift_limits->second.min_position;
    lift_max_position_ = lift_limits->second.max_position;
  }

  return CallbackReturn::SUCCESS;
}

double TrossenBaseHardwareInterface::get_command_or_zero(const std::string & interface_name) const
{
  const double value = get_command(interface_name);
  return std::isnan(value) ? 0.0 : value;
}

CallbackReturn
TrossenBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
  if (mock_) {
    RCLCPP_INFO(get_logger(), "Mock mode: not connecting to the Trossen Base.");
    return CallbackReturn::SUCCESS;
  }

  RCLCPP_INFO(get_logger(), "Configuring the Trossen Base Driver...");

  base_driver_ = std::make_unique<trossen_base::TrossenBase>();

  try {
    base_driver_->configure(false, false);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to configure TrossenBaseDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "TrossenBaseDriver configured.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
  // Update the state of the robot
  this->read(rclcpp::Time(0.0), rclcpp::Duration(0, 0));

  RCLCPP_INFO(get_logger(), "TrossenBaseDriver enabled.");

  return CallbackReturn::SUCCESS;
}

return_type TrossenBaseHardwareInterface::read(
  const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (mock_) {
    return read_mock(period);
  }

  trossen_base::PositionVelocityAcceleration odometry = base_driver_->get_odometry();

  const trossen_base::LiftState lift = base_driver_->get_lift_state();
  const trossen_base::ImuState imu = base_driver_->get_imu_state();
  const trossen_base::WheelState wheels = base_driver_->get_wheel_state();
  const trossen_base::BatteryState battery = base_driver_->get_battery_state();

  // Base (gpio), by interface index: position x/y/yaw in odom frame (m, rad),
  // velocity x/y/yaw in base frame (m/s, rad/s)
  const ComponentInfo & base = info_.gpios[INDEX_BASE_];
  for (size_t i = 0; i < 3; ++i) {
    set_state(base.name + "/" + base.state_interfaces[i].name, odometry.position[i]);
    set_state(base.name + "/" + base.state_interfaces[3 + i].name, odometry.velocity[i]);
  }

  // Lift: position (m) above bottom soft bound, velocity (m/s), positive = up
  const std::string & lift_joint = info_.joints[INDEX_LIFT_].name;
  set_state(lift_joint + "/" + HW_IF_POSITION, lift.position);
  set_state(lift_joint + "/" + HW_IF_VELOCITY, lift.velocity);

  // Swerve modules: pivot angle (rad); drive velocity m/s -> rad/s.
  // Drive position is not tracked and stays at its initial value.
  const std::string & fl_pivot = info_.joints[INDEX_FRONT_LEFT_PIVOT_].name;
  const std::string & fl_drive = info_.joints[INDEX_FRONT_LEFT_DRIVE_].name;
  const std::string & fr_pivot = info_.joints[INDEX_FRONT_RIGHT_PIVOT_].name;
  const std::string & fr_drive = info_.joints[INDEX_FRONT_RIGHT_DRIVE_].name;
  const std::string & b_pivot = info_.joints[INDEX_BACK_PIVOT_].name;
  const std::string & b_drive = info_.joints[INDEX_BACK_DRIVE_].name;
  set_state(fl_pivot + "/" + HW_IF_POSITION, wheels.front_left_pivot_angle);
  set_state(fl_drive + "/" + HW_IF_VELOCITY, wheels.front_left_velocity / wheel_radius_);
  set_state(fr_pivot + "/" + HW_IF_POSITION, wheels.front_right_pivot_angle);
  set_state(fr_drive + "/" + HW_IF_VELOCITY, wheels.front_right_velocity / wheel_radius_);
  set_state(b_pivot + "/" + HW_IF_POSITION, wheels.back_pivot_angle);
  set_state(b_drive + "/" + HW_IF_VELOCITY, wheels.back_velocity / wheel_radius_);

  // IMU (sensor), by interface index, in imu_link frame: orientation x/y/z/w from yaw only
  // (roll/pitch = 0), angular_velocity x/y/z (rad/s), linear_acceleration x/y/z
  // (m/s^2, z includes gravity)
  const ComponentInfo & imu_sensor = info_.sensors[INDEX_IMU_];
  const double imu_values[NUM_IMU_STATES_] = {
    0.0, 0.0, std::sin(imu.yaw / 2.0), std::cos(imu.yaw / 2.0),
    imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2],
    imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]};
  for (size_t i = 0; i < NUM_IMU_STATES_; ++i) {
    set_state(imu_sensor.name + "/" + imu_sensor.state_interfaces[i].name, imu_values[i]);
  }

  // Battery (sensor), by interface index: voltage (V), current (A, positive = discharging),
  // percentage (0-100), temperature (deg C), power_supply_status.
  // Driver charging_state: 0=stationary, 1=charging, 2=discharging.
  // sensor_msgs/BatteryState: 0=UNKNOWN, 1=CHARGING, 2=DISCHARGING, 3=NOT_CHARGING.
  double power_supply_status = 0.0;
  switch (battery.charging_state) {
    case 0: power_supply_status = 3.0; break;
    case 1: power_supply_status = 1.0; break;
    case 2: power_supply_status = 2.0; break;
    default: break;  // UNKNOWN
  }
  const ComponentInfo & battery_sensor = info_.sensors[INDEX_BATTERY_];
  const double battery_values[NUM_BATTERY_STATES_] = {
    battery.voltage, battery.current, battery.percent, battery.temperature, power_supply_status};
  for (size_t i = 0; i < NUM_BATTERY_STATES_; ++i) {
    set_state(battery_sensor.name + "/" + battery_sensor.state_interfaces[i].name,
      battery_values[i]);
  }

  return return_type::OK;
}

return_type TrossenBaseHardwareInterface::read_mock(const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Base (gpio): velocity commands are in the base frame, so rotate by yaw before integrating
  // into the odom-frame pose. Velocity states mirror the commands.
  const ComponentInfo & base = info_.gpios[INDEX_BASE_];
  std::array<double, 3> base_velocity{};
  for (size_t i = 0; i < NUM_BASE_COMMANDS_; ++i) {
    base_velocity[i] = get_command_or_zero(base.name + "/" + base.command_interfaces[i].name);
  }
  const double yaw = mock_base_position_[2];
  mock_base_position_[0] +=
    (base_velocity[0] * std::cos(yaw) - base_velocity[1] * std::sin(yaw)) * dt;
  mock_base_position_[1] +=
    (base_velocity[0] * std::sin(yaw) + base_velocity[1] * std::cos(yaw)) * dt;
  mock_base_position_[2] += base_velocity[2] * dt;
  for (size_t i = 0; i < 3; ++i) {
    set_state(base.name + "/" + base.state_interfaces[i].name, mock_base_position_[i]);
    set_state(base.name + "/" + base.state_interfaces[3 + i].name, base_velocity[i]);
  }

  // Lift: integrate the velocity command, clamped to the URDF position limits
  const std::string & lift_joint = info_.joints[INDEX_LIFT_].name;
  double lift_velocity = get_command_or_zero(lift_joint + "/" + HW_IF_VELOCITY);
  const double lift_target = mock_lift_position_ + lift_velocity * dt;
  mock_lift_position_ = std::clamp(lift_target, lift_min_position_, lift_max_position_);
  if (mock_lift_position_ != lift_target) {
    lift_velocity = 0.0;  // stopped at a limit
  }
  set_state(lift_joint + "/" + HW_IF_POSITION, mock_lift_position_);
  set_state(lift_joint + "/" + HW_IF_VELOCITY, lift_velocity);

  // Swerve modules: pivots and drives stay at their initial values

  // IMU (sensor): yaw from the mock base, yaw rate from the command, gravity on z
  const ComponentInfo & imu_sensor = info_.sensors[INDEX_IMU_];
  const double imu_values[NUM_IMU_STATES_] = {
    0.0, 0.0, std::sin(mock_base_position_[2] / 2.0), std::cos(mock_base_position_[2] / 2.0),
    0.0, 0.0, base_velocity[2],
    0.0, 0.0, 9.81};
  for (size_t i = 0; i < NUM_IMU_STATES_; ++i) {
    set_state(imu_sensor.name + "/" + imu_sensor.state_interfaces[i].name, imu_values[i]);
  }

  return return_type::OK;
}

return_type TrossenBaseHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  // Mock mode: commands are read back in read_mock(), nothing to send
  if (mock_) {
    return return_type::OK;
  }

  // Base (gpio), by interface index: velocity x/y/yaw in base frame (m/s, rad/s).
  // Commands start as NaN until a controller writes, so treat NaN as 0.
  const ComponentInfo & base = info_.gpios[INDEX_BASE_];
  std::array<double, 3> base_velocity{};
  for (size_t i = 0; i < NUM_BASE_COMMANDS_; ++i) {
    base_velocity[i] = get_command_or_zero(base.name + "/" + base.command_interfaces[i].name);
  }
  if (!base_driver_->set_velocities(base_velocity)) {
    RCLCPP_WARN(get_logger(), "Failed to send velocity command.");
  }

  // Lift velocity (m/s), positive = up
  const double lift_velocity =
    get_command_or_zero(info_.joints[INDEX_LIFT_].name + "/" + HW_IF_VELOCITY);
  if (!base_driver_->set_lift_velocity(lift_velocity)) {
    RCLCPP_WARN(get_logger(), "Failed to send lift velocity command.");
  }
  return return_type::OK;
}

CallbackReturn TrossenBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  if (base_driver_) {
    if (!base_driver_->set_velocities({0.0, 0.0, 0.0})) {
      RCLCPP_WARN(get_logger(), "Failed to send zero-velocity command on deactivate");
    }
    if (!base_driver_->set_lift_velocity(0.0)) {
      RCLCPP_WARN(get_logger(), "Failed to send zero lift velocity command on deactivate");
    }
  }

  RCLCPP_INFO(get_logger(), "TrossenBaseDriver stopped.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn TrossenBaseHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (base_driver_) {
    base_driver_->cleanup();
    base_driver_.reset();
  }

  return CallbackReturn::SUCCESS;
}

return_type TrossenBaseHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Zero releases interfaces so write() stops the base/lift
  for (const std::string & name : stop_interfaces) {
    set_command(name, 0.0);
  }
  // Zero newly claimed interfaces so doesn't inherit stale value
  for (const std::string & name : start_interfaces) {
    set_command(name, 0.0);
  }
  return return_type::OK;
}

}  // namespace trossen_base_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_base_hardware::TrossenBaseHardwareInterface,
  hardware_interface::SystemInterface)
