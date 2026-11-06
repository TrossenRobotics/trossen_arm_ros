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

#include <cmath>

namespace trossen_base_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
TrossenBaseHardwareInterface::on_init(const HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
TrossenBaseHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> state_interfaces;

  // position_* in world frame (m, rad); velocity_* in base frame (m/s, rad/s);
  // lift_position = height above bottom soft bound (m), lift_velocity in m/s, positive = up;
  // *_velocity per swerve module drive (m/s), *_pivot_angle per swerve module (rad);
  // battery_* named for battery_state_broadcaster: battery_voltage (V),
  // battery_current (A, positive = discharging), battery_percentage (0-100),
  // battery_temperature (deg C), battery_power_supply_status
  for (const char * name : {
      "position_x", "position_y", "position_yaw",
      "velocity_x", "velocity_y", "velocity_yaw",
      "lift_position", "lift_velocity",
      "front_left_velocity", "front_left_pivot_angle",
      "front_right_velocity", "front_right_pivot_angle",
      "back_velocity", "back_pivot_angle",
      "battery_voltage", "battery_current", "battery_percentage",
      "battery_temperature", "battery_power_supply_status"})
  {
    InterfaceInfo interface_info;
    interface_info.name = name;
    interface_info.data_type = "double";
    state_interfaces.push_back(InterfaceDescription(info_.name, interface_info));
  }

  // IMU, named for imu_sensor_broadcaster (sensor_name = <name>_imu), in imu frame:
  // orientation quaternion (yaw only, roll/pitch = 0), angular_velocity (rad/s),
  // linear_acceleration (m/s^2, z includes gravity)
  for (const char * name : {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    InterfaceInfo interface_info;
    interface_info.name = name;
    interface_info.data_type = "double";
    state_interfaces.push_back(InterfaceDescription(info_.name + "_imu", interface_info));
  }

  return state_interfaces;
}

std::vector<hardware_interface::InterfaceDescription>
TrossenBaseHardwareInterface::export_unlisted_command_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> command_interfaces;

  // velocity_x/y in m/s, velocity_yaw in rad/s (base frame); lift_velocity in m/s, positive = up
  for (const char * name : {"velocity_x", "velocity_y", "velocity_yaw", "lift_velocity"}) {
    InterfaceInfo interface_info;
    interface_info.name = name;
    interface_info.data_type = "double";
    command_interfaces.push_back(InterfaceDescription(info_.name, interface_info));
  }

  return command_interfaces;
}

CallbackReturn
TrossenBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
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

return_type TrossenBaseHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  trossen_base::PositionVelocityAcceleration odometry = base_driver_->get_odometry();

  const trossen_base::LiftState lift = base_driver_->get_lift_state();
  const trossen_base::ImuState imu = base_driver_->get_imu_state();
  const trossen_base::WheelState wheels = base_driver_->get_wheel_state();
  const trossen_base::BatteryState battery = base_driver_->get_battery_state();

  const std::string & n = info_.name;
  set_state(n + "/position_x", odometry.position[0]);
  set_state(n + "/position_y", odometry.position[1]);
  set_state(n + "/position_yaw", odometry.position[2]);
  set_state(n + "/velocity_x", odometry.velocity[0]);
  set_state(n + "/velocity_y", odometry.velocity[1]);
  set_state(n + "/velocity_yaw", odometry.velocity[2]);
  set_state(n + "/lift_position", lift.position);
  set_state(n + "/lift_velocity", lift.velocity);
  const std::string imu_n = n + "_imu";
  set_state(imu_n + "/orientation.x", 0.0);
  set_state(imu_n + "/orientation.y", 0.0);
  set_state(imu_n + "/orientation.z", std::sin(imu.yaw / 2.0));
  set_state(imu_n + "/orientation.w", std::cos(imu.yaw / 2.0));
  set_state(imu_n + "/angular_velocity.x", imu.angular_velocity[0]);
  set_state(imu_n + "/angular_velocity.y", imu.angular_velocity[1]);
  set_state(imu_n + "/angular_velocity.z", imu.angular_velocity[2]);
  set_state(imu_n + "/linear_acceleration.x", imu.linear_acceleration[0]);
  set_state(imu_n + "/linear_acceleration.y", imu.linear_acceleration[1]);
  set_state(imu_n + "/linear_acceleration.z", imu.linear_acceleration[2]);
  set_state(n + "/front_left_velocity", wheels.front_left_velocity);
  set_state(n + "/front_left_pivot_angle", wheels.front_left_pivot_angle);
  set_state(n + "/front_right_velocity", wheels.front_right_velocity);
  set_state(n + "/front_right_pivot_angle", wheels.front_right_pivot_angle);
  set_state(n + "/back_velocity", wheels.back_velocity);
  set_state(n + "/back_pivot_angle", wheels.back_pivot_angle);
  set_state(n + "/battery_voltage", battery.voltage);
  set_state(n + "/battery_current", battery.current);
  set_state(n + "/battery_percentage", battery.percent);
  set_state(n + "/battery_temperature", battery.temperature);

  // Driver: 0=stationary, 1=charging, 2=discharging.
  // sensor_msgs/BatteryState: 0=UNKNOWN, 1=CHARGING, 2=DISCHARGING, 3=NOT_CHARGING.
  double power_supply_status = 0.0;
  switch (battery.charging_state) {
    case 0: power_supply_status = 3.0; break;
    case 1: power_supply_status = 1.0; break;
    case 2: power_supply_status = 2.0; break;
    default: break;  // UNKNOWN
  }
  set_state(n + "/battery_power_supply_status", power_supply_status);


  return return_type::OK;
}

return_type TrossenBaseHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  auto cmd = [&](const std::string & n) {
    double v = get_command(info_.name + "/" + n);
    return std::isnan(v) ? 0.0 : v;   // commands start as NaN until a controller writes
  };

  if (!base_driver_->set_velocities({cmd("velocity_x"), cmd("velocity_y"), cmd("velocity_yaw")})) {
    RCLCPP_WARN(get_logger(), "Failed to send velocity command.");
  }
  if (!base_driver_->set_lift_velocity(cmd("lift_velocity"))) {
    RCLCPP_WARN(get_logger(), "Failed to send lift velocity command.");
  }
  return return_type::OK;
}

CallbackReturn TrossenBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  if (!base_driver_->set_velocities({0.0, 0.0, 0.0})) {
    RCLCPP_WARN(get_logger(), "Failed to send zero-velocity command on deactivate");
  }
  if (!base_driver_->set_lift_velocity(0.0)) {
    RCLCPP_WARN(get_logger(), "Failed to send zero lift velocity command on deactivate");
  }

  RCLCPP_INFO(get_logger(), "TrossenBaseDriver stopped.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenBaseHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (base_driver_) {
    base_driver_->cleanup();
    base_driver_.reset();
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace trossen_base_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_base_hardware::TrossenBaseHardwareInterface,
  hardware_interface::SystemInterface)
