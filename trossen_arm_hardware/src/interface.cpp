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

#include "trossen_arm_hardware/interface.hpp"

namespace trossen_arm_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
TrossenArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get robot model
  try {
    robot_model_ = trossen_arm::Model(std::stoi(info.hardware_parameters.at("robot_model")));
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'robot_model' set to '%d'.",
      static_cast<int>(robot_model_));
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Required parameter 'robot_model' not specified.");
    return CallbackReturn::FAILURE;
  } catch (const std::invalid_argument & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid 'robot_model' value specified: '%d'.", static_cast<int>(robot_model_));
    return CallbackReturn::FAILURE;
  }

  // Get robot end effector
  try {
    end_effector_str_ = info.hardware_parameters.at("end_effector");
    if (end_effector_str_ == END_EFFECTOR_BASE) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_base;
    } else if (end_effector_str_ == END_EFFECTOR_FOLLOWER) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_follower;
    } else if (end_effector_str_ == END_EFFECTOR_LEADER) {
      end_effector_ = trossen_arm::StandardEndEffector::wxai_v0_leader;
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid 'end_effector' value specified: '%s'.", end_effector_str_.c_str());
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'end_effector' set to '%s'.", end_effector_str_.c_str());
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Required parameter 'end_effector' not specified.");
    return CallbackReturn::FAILURE;
  }

  // Get robot IP address
  try {
    driver_ip_address_ = info.hardware_parameters.at("ip_address");
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'ip_address' set to '%s'.",
      driver_ip_address_.c_str());
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Parameter 'ip_address' not specified. Defaulting to '%s'.",
      DRIVER_IP_ADDRESS_DEFAULT);
    driver_ip_address_ = DRIVER_IP_ADDRESS_DEFAULT;
  }

  // Joint state interfaces
  joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Joint command interfaces
  joint_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_external_effort_commands_.resize(info_.joints.size(),
      std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints) {
    // Each joint has 3 command interfaces: position, velocity, external effort (in that order)
    // Expect exactly three command interfaces
    if (joint.command_interfaces.size() != COUNT_COMMAND_INTERFACES_) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu command interfaces found. %zu expected.",
        joint.name.c_str(),
        joint.command_interfaces.size(),
        COUNT_COMMAND_INTERFACES_);
      return CallbackReturn::ERROR;
    }

    // Position first
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_POSITION_).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_POSITION_).name.c_str(),
        INDEX_COMMAND_INTERFACE_POSITION_,
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name.c_str(),
        INDEX_COMMAND_INTERFACE_VELOCITY_,
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // External effort third
    if (
      joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_).name !=
      HW_IF_EXTERNAL_EFFORT)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_).name.c_str(),
        INDEX_COMMAND_INTERFACE_EXTERNAL_EFFORT_,
        HW_IF_EXTERNAL_EFFORT);
      return CallbackReturn::ERROR;
    }

    // Each joint has 3 state interfaces: position, velocity, effort (in that order)
    // Expect exactly three state interfaces
    if (joint.state_interfaces.size() != COUNT_STATE_INTERFACES_) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu state interfaces found. %zu expected.",
        joint.name.c_str(),
        joint.state_interfaces.size(),
        COUNT_STATE_INTERFACES_);
      return CallbackReturn::ERROR;
    }

    // Position first
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_POSITION_).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_POSITION_).name.c_str(),
        INDEX_STATE_INTERFACE_POSITION_,
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name.c_str(),
        INDEX_STATE_INTERFACE_VELOCITY_,
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Effort third
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found at index '%ld'. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name.c_str(),
        INDEX_STATE_INTERFACE_EFFORT_,
        HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  // Initialize Cartesian pose command interfaces
  cartesian_pose_commands_.resize(6, std::numeric_limits<double>::quiet_NaN());

  // Validate GPIO interfaces for Cartesian control
  if (info_.gpios.size() != 6) {
    RCLCPP_ERROR(
      get_logger(),
      "Expected 6 GPIO interfaces for Cartesian control, found %zu.",
      info_.gpios.size());
    return CallbackReturn::ERROR;
  }

  // Expected GPIO names
  const std::vector<std::string> expected_gpio_names = {
    "ee_pos_x", "ee_pos_y", "ee_pos_z", "ee_rot_x", "ee_rot_y", "ee_rot_z"
  };

  for (size_t i = 0; i < expected_gpio_names.size(); i++) {
    if (info_.gpios[i].name != expected_gpio_names[i]) {
      RCLCPP_ERROR(
        get_logger(),
        "GPIO interface at index %zu has name '%s', expected '%s'.",
        i,
        info_.gpios[i].name.c_str(),
        expected_gpio_names[i].c_str());
      return CallbackReturn::ERROR;
    }

    // Verify each GPIO has exactly one command interface named 'cartesian_pose'
    if (info_.gpios[i].command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        get_logger(),
        "GPIO '%s' has %zu command interfaces, expected 1.",
        info_.gpios[i].name.c_str(),
        info_.gpios[i].command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (info_.gpios[i].command_interfaces[0].name != HW_IF_CARTESIAN_POSE) {
      RCLCPP_ERROR(
        get_logger(),
        "GPIO '%s' has command interface '%s', expected '%s'.",
        info_.gpios[i].name.c_str(),
        info_.gpios[i].command_interfaces[0].name.c_str(),
        HW_IF_CARTESIAN_POSE);
      return CallbackReturn::ERROR;
    }
  }

  // Parse cartesian_goal_time parameter
  if (info_.hardware_parameters.find("cartesian_goal_time") != info_.hardware_parameters.end()) {
    cartesian_goal_time_ = std::stod(info_.hardware_parameters.at("cartesian_goal_time"));
    RCLCPP_INFO(
      get_logger(),
      "Cartesian goal time parameter set to: %.3f s",
      cartesian_goal_time_);
  } else {
    cartesian_goal_time_ = 0.02;
    RCLCPP_INFO(
      get_logger(),
      "Using default cartesian goal time: %.3f s",
      cartesian_goal_time_);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TrossenArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Position state interfaces
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        HW_IF_POSITION,
        &joint_positions_[i]));
    // Velocity state interfaces
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        HW_IF_VELOCITY,
        &joint_velocities_[i]));
    // Effort state interfaces
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        HW_IF_EFFORT,
        &joint_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrossenArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // Position command interfaces
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        HW_IF_POSITION,
        &joint_position_commands_[i]));
    // Velocity command interfaces
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        HW_IF_VELOCITY,
        &joint_velocity_commands_[i]));
    // External effort command interfaces
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        HW_IF_EXTERNAL_EFFORT,
        &joint_external_effort_commands_[i]));
  }

  // Export Cartesian pose command interfaces from GPIO definitions
  // GPIO names and order were already validated in on_init()
  for (size_t i = 0; i < info_.gpios.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.gpios[i].name,
        HW_IF_CARTESIAN_POSE,
        &cartesian_pose_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn
TrossenArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring the Trossen Arm Driver...");
  try {
    arm_driver_ = std::make_unique<TrossenArmDriver>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create TrossenArmDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!arm_driver_) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create TrossenArmDriver.");
    return CallbackReturn::ERROR;
  }

  try {
    arm_driver_->configure(
      robot_model_,
      end_effector_,
      driver_ip_address_.c_str(),
      true);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to configure TrossenArmDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Update the robot output
  robot_output_ = arm_driver_->get_robot_output();

  RCLCPP_INFO(
    get_logger(),
    "TrossenArmDriver configured with model %d, IP Address '%s', End Effector '%s'.",
    static_cast<int>(robot_model_),
    driver_ip_address_.c_str(),
    end_effector_str_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  first_update_ = true;

  // Update the state of the robot
  this->read(rclcpp::Time(0.0), rclcpp::Duration(0, 0));

  RCLCPP_INFO(get_logger(), "TrossenArmDriver enabled.");

  return CallbackReturn::SUCCESS;
}

return_type
TrossenArmHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  robot_output_ = arm_driver_->get_robot_output();

  // Get joint positions
  joint_positions_ = robot_output_.joint.all.positions;

  // Get joint velocities
  joint_velocities_ = robot_output_.joint.all.velocities;

  // Get joint efforts
  joint_efforts_ = robot_output_.joint.all.efforts;

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // If first time writing to the hardware, set all commands to the current positions
  if (first_update_) {
    RCLCPP_DEBUG(
      get_logger(),
      "First write update. Setting joint position commands to current positions.");
    joint_position_commands_ = joint_positions_;
    first_update_ = false;
  }

  // Send the now validated commands to the driver
  if (arm_position_mode_running_) {
    arm_driver_->set_all_positions(joint_position_commands_, 0.0, false);
  } else if (arm_velocity_mode_running_) {
    RCLCPP_ERROR(get_logger(), "Velocity mode not implemented yet.");
    return return_type::ERROR;
    // arm_driver_->set_all_velocities(joint_velocity_commands_, 0.0,
  } else if (arm_external_effort_mode_running_) {
    arm_driver_->set_all_external_efforts(joint_external_effort_commands_, 0.0, false);
  }

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // If nothing new requested, nothing to validate
  if (start_interfaces.empty()) {
    RCLCPP_DEBUG(get_logger(), "No start interfaces requested. Nothing to prepare.");
    return return_type::OK;
  }

  // Determine which single command interface type (position/velocity/external effort) is requested
  std::string requested_interface_type;
  for (const auto & iface : start_interfaces) {
    // Interface names are in the format: `<joint_name>/<interface_type>`
    auto slash_pos = iface.rfind('/');
    std::string type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);

    if (type != HW_IF_POSITION && type != HW_IF_VELOCITY && type != HW_IF_EXTERNAL_EFFORT) {
      RCLCPP_ERROR(get_logger(), "Unsupported command interface '%s' requested.", type.c_str());
      return return_type::ERROR;
    }

    if (requested_interface_type.empty()) {
      requested_interface_type = type;
    } else if (requested_interface_type != type) {
      RCLCPP_ERROR(
        get_logger(),
        "Mixed command interface types requested in a single mode switch: '%s' and '%s'.",
        requested_interface_type.c_str(), type.c_str());
      return return_type::ERROR;
    }
  }

  // Validate transitions. Only one control mode can be active at a time.
  // If a different mode is already running, its interfaces must be listed in stop_interfaces.

  if (requested_interface_type == HW_IF_POSITION) {
    // Validate position mode can be started - need to stop velocity, external effort interfaces
    if (arm_position_mode_running_) {
      // No change needed
      RCLCPP_DEBUG(get_logger(), "Position mode already active. No change needed.");
      return return_type::OK;
    }
    if (arm_velocity_mode_running_ && !interface_type_in_stop(stop_interfaces, HW_IF_VELOCITY)) {
      RCLCPP_ERROR(
        get_logger(),
        "Velocity mode is active but not requested to stop before switching to position mode.");
      return return_type::ERROR;
    }
    if (
      arm_external_effort_mode_running_ && !interface_type_in_stop(stop_interfaces,
        HW_IF_EXTERNAL_EFFORT))
    {
      RCLCPP_ERROR(
        get_logger(),
        "External effort mode is active but not requested to stop before switching to "
        "position mode.");
      return return_type::ERROR;
    }
  } else if (requested_interface_type == HW_IF_VELOCITY) {
    // Validate velocity mode can be started - need to stop position, external effort interfaces
    if (arm_velocity_mode_running_) {
      // No change needed
      RCLCPP_DEBUG(get_logger(), "Velocity mode already active. No change needed.");
      return return_type::OK;
    }
    if (arm_position_mode_running_ && !interface_type_in_stop(stop_interfaces, HW_IF_POSITION)) {
      RCLCPP_ERROR(
        get_logger(),
        "Position mode is active but not requested to stop before switching to velocity mode.");
      return return_type::ERROR;
    }
    if (
      arm_external_effort_mode_running_ && !interface_type_in_stop(stop_interfaces,
        HW_IF_EXTERNAL_EFFORT))
    {
      RCLCPP_ERROR(
        get_logger(),
        "External effort mode is active but not requested to stop before switching to "
        "velocity mode.");
      return return_type::ERROR;
    }
    // TODO(lukeschmtit-tr): Velocity mode not implemented yet - handle at prepare
    RCLCPP_ERROR(get_logger(), "Velocity mode requested but not implemented.");
    return return_type::ERROR;
  } else if (requested_interface_type == HW_IF_EXTERNAL_EFFORT) {
    // Validate external effort mode can be started - need to stop position, velocity interfaces
    if (arm_external_effort_mode_running_) {
      // No change needed
      RCLCPP_DEBUG(get_logger(), "External effort mode already active. No change needed.");
      return return_type::OK;
    }
    if (arm_position_mode_running_ && !interface_type_in_stop(stop_interfaces, HW_IF_POSITION)) {
      RCLCPP_ERROR(
        get_logger(),
        "Position mode is active but not requested to stop before switching to "
        "external effort mode.");
      return return_type::ERROR;
    }
    if (arm_velocity_mode_running_ && !interface_type_in_stop(stop_interfaces, HW_IF_VELOCITY)) {
      RCLCPP_ERROR(
        get_logger(),
        "Velocity mode is active but not requested to stop before switching to "
        "external effort mode.");
      return return_type::ERROR;
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Command mode switch preparation successful. New mode requested: '%s'.",
    requested_interface_type.c_str());

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Determine which command interface types are being started and stopped
  auto stop_types = interface_types_from_list(stop_interfaces);
  auto start_types = interface_types_from_list(start_interfaces);

  // Stop requested modes
  if (stop_types.count(HW_IF_POSITION)) {
    arm_position_mode_running_ = false;
  }
  if (stop_types.count(HW_IF_VELOCITY)) {
    arm_velocity_mode_running_ = false;
  }
  if (stop_types.count(HW_IF_EXTERNAL_EFFORT)) {
    arm_external_effort_mode_running_ = false;
  }

  // Start requested mode (only one should be present due to prepare validation)
  if (start_types.count(HW_IF_POSITION)) {
    arm_position_mode_running_ = true;
    arm_velocity_mode_running_ = false;
    arm_external_effort_mode_running_ = false;
    joint_position_commands_ = joint_positions_;
    try {
      arm_driver_->set_all_modes(trossen_arm::Mode::position);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to set driver to position mode: %s", e.what());
      return return_type::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Switched to position command mode.");
  } else if (start_types.count(HW_IF_VELOCITY)) {
    // Velocity not implemented yet
    RCLCPP_ERROR(get_logger(), "Velocity mode requested but not implemented.");
    return return_type::ERROR;
  } else if (start_types.count(HW_IF_EXTERNAL_EFFORT)) {
    arm_position_mode_running_ = false;
    arm_velocity_mode_running_ = false;
    arm_external_effort_mode_running_ = true;
    std::fill(joint_external_effort_commands_.begin(), joint_external_effort_commands_.end(), 0.0);
    try {
      arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to set driver to external effort mode: %s", e.what());
      return return_type::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Switched to external effort command mode.");
  }

  // If no start interfaces provided we may just be stopping a mode
  if (start_types.empty()) {
    if (!arm_position_mode_running_ && !arm_velocity_mode_running_ &&
      !arm_external_effort_mode_running_)
    {
      try {
        arm_driver_->set_all_modes(trossen_arm::Mode::idle);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to set driver to idle mode: %s", e.what());
        return return_type::ERROR;
      }
      RCLCPP_INFO(get_logger(), "All command modes stopped. Driver set to idle.");
    }
  }

  return return_type::OK;
}

CallbackReturn
TrossenArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_driver_->set_all_modes(trossen_arm::Mode::idle);

  RCLCPP_INFO(get_logger(), "TrossenArmDriver disabled.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_output_ = trossen_arm::RobotOutput();
  arm_driver_.reset();

  return CallbackReturn::SUCCESS;
}

bool
TrossenArmHardwareInterface::interface_type_in_stop(
  const std::vector<std::string> & stop_interfaces,
  const std::string & type)
{
  for (const auto & iface : stop_interfaces) {
    auto slash_pos = iface.rfind('/');
    std::string stop_type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    if (stop_type == type) {
      return true;
    }
  }
  return false;
}

std::set<std::string>
TrossenArmHardwareInterface::interface_types_from_list(const std::vector<std::string> & ifaces)
{
  std::set<std::string> types;
  for (const auto & iface : ifaces) {
    auto slash_pos = iface.rfind('/');
    std::string type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    types.insert(type);
  }
  return types;
}

}  // namespace trossen_arm_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_hardware::TrossenArmHardwareInterface,
  hardware_interface::SystemInterface)
