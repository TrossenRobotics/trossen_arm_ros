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

  // Get the gripper joint index, default to the last joint if not specified
  // TODO(lukeschmitt-tr): Handle no or custom gripper cases
  try {
    gripper_joint_index_ = std::stoi(info.hardware_parameters.at("gripper_joint_index"));
    // Validatet the index
    if (gripper_joint_index_ < 0 || gripper_joint_index_ >= static_cast<int>(info_.joints.size())) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid 'gripper_joint_index' value specified: '%d'. Must be between 0 and %zu.",
        gripper_joint_index_,
        info_.joints.size() - 1);
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'gripper_joint_index' set to '%d'.",
      gripper_joint_index_);
  } catch (const std::out_of_range & /*e*/) {
    // Default to the last joint
    gripper_joint_index_ = static_cast<int>(info_.joints.size()) - 1;
    RCLCPP_WARN(
      get_logger(),
      "Parameter 'gripper_joint_index' not specified. Defaulting to last joint: '%d'.",
      gripper_joint_index_);
  } catch (const std::invalid_argument & /*e*/) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid 'gripper_joint_index' value specified: '%s'.",
      info.hardware_parameters.at("gripper_joint_index").c_str());
    return CallbackReturn::FAILURE;
  }

  // Joint state interfaces
  joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Joint command interfaces
  joint_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints) {
    // Each joint has 3 command interfaces: position, velocity, effort (in that order)
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
        "Joint '%s' has '%s' command interface found. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_POSITION_).name.c_str(),
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_VELOCITY_).name.c_str(),
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Effort third
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EFFORT_).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_EFFORT_).name.c_str(),
        HW_IF_EFFORT);
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
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_POSITION_).name.c_str(),
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_VELOCITY_).name.c_str(),
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Effort third
    if (joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(INDEX_STATE_INTERFACE_EFFORT_).name.c_str(),
        HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  // Build the joint name to index map
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_name_to_index_[info_.joints[i].name] = i;
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
    // Effort command interfaces
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        HW_IF_EFFORT,
        &joint_effort_commands_[i]));
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

  // TODO(lukeschmitt-tr): Validate commands against NaN and inf

  // Send the arm commands to the driver
  switch (arm_mode_){
    case CommandMode::POSITION: {
      std::vector<double> arm_position_commands(
        joint_position_commands_.begin(),
        joint_position_commands_.begin() + gripper_joint_index_);
      arm_driver_->set_arm_positions(arm_position_commands, 0.0, false);
      break;
    };
    case CommandMode::VELOCITY: {
      RCLCPP_ERROR(get_logger(), "Velocity mode not implemented yet.");
      return return_type::ERROR;
      break;
    };
    case CommandMode::EFFORT: {
      std::vector<double> arm_external_effort_commands(
        joint_effort_commands_.begin(),
        joint_effort_commands_.begin() + gripper_joint_index_);
      arm_driver_->set_arm_external_efforts(arm_external_effort_commands, 0.0, false);
      break;
    };
  }

  // Send the gripper commands to the driver
  switch (gripper_mode_) {
    case CommandMode::POSITION: {
      arm_driver_->set_gripper_position(
        joint_position_commands_.at(gripper_joint_index_), 0.0, false);
      break;
    }
    case CommandMode::VELOCITY: {
      arm_driver_->set_gripper_velocity(
        joint_velocity_commands_.at(gripper_joint_index_), 0.0, false);
      break;
    }
    case CommandMode::EFFORT: {
      // Effort is used directly for the gripper's effort mode
      arm_driver_->set_gripper_effort(
        joint_effort_commands_.at(gripper_joint_index_), 0.0, false);
      break;
    }
  }

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Differentiate between arm and gripper interfaces
  // Arm interfaces are all except the gripper joint index
  auto is_gripper_interface = [this](const std::string & iface) {
    auto slash_pos = iface.rfind('/');
    std::string joint_name = (slash_pos == std::string::npos) ? iface : iface.substr(0, slash_pos);
    auto it = joint_name_to_index_.find(joint_name);
    if (it == joint_name_to_index_.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown joint name '%s' in interface '%s'.",
        joint_name.c_str(),
        iface.c_str());
      return false;
    }
    return it->second == gripper_joint_index_;
  };

  // Get the start and stop interfaces for the arm and gripper separately
  std::vector<std::string> arm_start_interfaces;
  std::vector<std::string> gripper_start_interfaces;
  for (const auto & iface : start_interfaces) {
    if (is_gripper_interface(iface)) {
      gripper_start_interfaces.push_back(iface);
    } else {
      arm_start_interfaces.push_back(iface);
    }
  }
  std::vector<std::string> arm_stop_interfaces;
  std::vector<std::string> gripper_stop_interfaces;
  for (const auto & iface : stop_interfaces) {
    if (is_gripper_interface(iface)) {
      gripper_stop_interfaces.push_back(iface);
    } else {
      arm_stop_interfaces.push_back(iface);
    }
  }

  // Determine which single command interface type (position/velocity/effort) is requested for the
  // joints in the arm. If multiple types are requested for joints in the arm, return an error.
  std::string requested_interface_type_arm;
  for (const auto & iface : arm_start_interfaces) {
    std::string joint_name, type;
    if (!validate_command_interface(iface, joint_name, type)) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid interface name '%s' requested.",
        iface.c_str());
      return return_type::ERROR;
    }

    // Check that the joint exists
    if (joint_name_to_index_.find(joint_name) == joint_name_to_index_.end()) {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown joint name '%s' in interface '%s'.",
        joint_name.c_str(),
        iface.c_str());
      return return_type::ERROR;
    }

    // Check that only one type is requested
    if (requested_interface_type_arm.empty()) {
      // If first type found, set it
      requested_interface_type_arm = type;
    } else if (requested_interface_type_arm != type) {
      // Error if different type found than first type
      RCLCPP_ERROR(
        get_logger(),
        "Mixed command interface types requested in a single mode switch for the arm: '%s' and '%s'.",
        requested_interface_type_arm.c_str(), type.c_str());
      return return_type::ERROR;
    }
  }

  // Determine which command interface type (position/velocity/effort) is requested for the gripper
  // joint. If multiple types are requested for the gripper joint, return an error.
  std::string requested_interface_type_gripper;
  if (gripper_start_interfaces.size() > 1) {
    RCLCPP_ERROR(
      get_logger(),
      "Multiple command interfaces requested for the gripper joint. Only one can be active at a time.");
    return return_type::ERROR;
  }
  if (!gripper_start_interfaces.empty()) {
    std::string requested_interface_gripper = gripper_start_interfaces.front();
    std::string joint_name, type;
    if (!validate_command_interface(requested_interface_gripper, joint_name, type)) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid interface name '%s' requested.",
        requested_interface_gripper.c_str());
      return return_type::ERROR;
    }
    requested_interface_type_gripper = type;
  }

  // Validate transitions for the arm. Only one control mode can be active at a time.
  // If a different mode is already running, its interfaces must be listed in arm_stop_interfaces.

  // Stop-only transition (no new start interfaces for arm but an active mode is requested to stop)
  if (requested_interface_type_arm.empty() && !arm_stop_interfaces.empty()) {
    // Determine if current arm mode is being stopped without a replacement
    std::string current_type = hardware_type_from_command_mode(arm_mode_);
    if (!current_type.empty()) {
      if (interface_type_in_stop(arm_stop_interfaces, current_type)) {
        RCLCPP_DEBUG(
          get_logger(),
          "Arm stop-only transition requested for active mode '%s' (will go idle).",
          current_type.c_str());
      }
    }
  }
  if (requested_interface_type_arm == HW_IF_POSITION) {
    // Validate position mode can be started - need to stop velocity, effort interfaces
    switch (arm_mode_) {
      case CommandMode::POSITION: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Arm position mode already active. No change needed.");
        break;

      }
      case CommandMode::VELOCITY: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_VELOCITY)) {
          RCLCPP_ERROR(
            get_logger(),
            "Velocity mode is active but not requested to stop before switching to position mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::EFFORT: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_EFFORT)) {
          RCLCPP_ERROR(
            get_logger(),
            "Effort mode is active but not requested to stop before switching to position mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
  } else if (requested_interface_type_arm == HW_IF_VELOCITY) {
    // Validate velocity mode can be started - need to stop position, effort interfaces
    switch (arm_mode_) {
      case CommandMode::VELOCITY: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Arm velocity mode already active. No change needed.");
        break;
      }
      case CommandMode::POSITION: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_POSITION)) {
          RCLCPP_ERROR(
            get_logger(),
            "Position mode is active but not requested to stop before switching to velocity mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::EFFORT: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_EFFORT)) {
          RCLCPP_ERROR(
            get_logger(),
            "Effort mode is active but not requested to stop before switching to velocity mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
    // TODO(lukeschmtit-tr): Velocity mode not implemented yet - handle at prepare
    RCLCPP_ERROR(get_logger(), "Velocity mode requested but not implemented.");
    return return_type::ERROR;
  } else if (requested_interface_type_arm == HW_IF_EFFORT) {
    // Validate effort mode can be started - need to stop position, velocity interfaces
    switch (arm_mode_) {
      case CommandMode::EFFORT: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Arm effort mode already active. No change needed.");
        break;
      }
      case CommandMode::POSITION: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_POSITION)) {
          RCLCPP_ERROR(
            get_logger(),
            "Arm position mode is active but not requested to stop before switching to effort mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::VELOCITY: {
        if (!interface_type_in_stop(arm_stop_interfaces, HW_IF_VELOCITY)) {
          RCLCPP_ERROR(
            get_logger(),
            "Arm velocity mode is active but not requested to stop before switching to effort mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
  }

  // Validate transitions for the gripper. Only one control mode can be active at a time.
  // If a different mode is already running, its interfaces must be listed in gripper_stop_interfaces.

  // Stop-only transition (no new start interface for gripper but an active mode is requested to stop)
  if (requested_interface_type_gripper.empty() && !gripper_stop_interfaces.empty()) {
    std::string current_type = hardware_type_from_command_mode(arm_mode_);
    if (!current_type.empty()) {
      if (interface_type_in_stop(gripper_stop_interfaces, current_type)) {
        RCLCPP_DEBUG(
          get_logger(),
          "Gripper stop-only transition requested for active mode '%s' (will go idle).",
          current_type.c_str());
      }
    }
  }
  if (requested_interface_type_gripper == HW_IF_POSITION) {
    // Validate position mode can be started - need to stop velocity, effort interfaces
    switch (gripper_mode_) {
      case CommandMode::POSITION: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Gripper position mode already active. No change needed.");
        break;
      }
      case CommandMode::VELOCITY: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_VELOCITY)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper velocity mode is active but not requested to stop before switching to position mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::EFFORT: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_EFFORT)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper effort mode is active but not requested to stop before switching to position mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
  } else if (requested_interface_type_gripper == HW_IF_VELOCITY) {
    // Validate velocity mode can be started - need to stop position, effort interfaces
    switch (gripper_mode_) {
      case CommandMode::VELOCITY: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Gripper velocity mode already active. No change needed.");
        break;
      }
      case CommandMode::POSITION: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_POSITION)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper position mode is active but not requested to stop before switching to velocity mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::EFFORT: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_EFFORT)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper effort mode is active but not requested to stop before switching to velocity mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
  } else if (requested_interface_type_gripper == HW_IF_EFFORT) {
    // Validate effort mode can be started - need to stop position, velocity interfaces
    switch (gripper_mode_) {
      case CommandMode::EFFORT: {
        // No change needed
        RCLCPP_DEBUG(get_logger(), "Gripper effort mode already active. No change needed.");
        break;
      }
      case CommandMode::POSITION: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_POSITION)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper position mode is active but not requested to stop before switching to effort mode.");
          return return_type::ERROR;
        }
        break;
      }
      case CommandMode::VELOCITY: {
        if (!interface_type_in_stop(gripper_stop_interfaces, HW_IF_VELOCITY)) {
          RCLCPP_ERROR(
            get_logger(),
            "Gripper velocity mode is active but not requested to stop before switching to effort mode.");
          return return_type::ERROR;
        }
        break;
      }
    }
  }

  // At this point we have validated:
  // - Only one command interface type is being requested for the arm
  // - The requested command interface type is valid given the current arm mode and any requested
  //   stop interfaces
  // - Only one command interface type is being requested for the gripper
  // - The requested command interface type is valid given the current gripper mode and any
  //   requested stop interfaces
  // We can now proceed to perform the command mode switch for the arm and gripper separately
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

  // Start requested mode (only one should be present due to prepare validation)
  if (start_types.count(HW_IF_POSITION)) {
    arm_mode_ = CommandMode::POSITION;
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
  } else if (start_types.count(HW_IF_EFFORT)) {
    arm_mode_ = CommandMode::EFFORT;
    std::fill(joint_effort_commands_.begin(), joint_effort_commands_.end(), 0.0);
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
    if (
      arm_mode_ != CommandMode::POSITION
      && arm_mode_ != CommandMode::VELOCITY
      && arm_mode_ != CommandMode::EFFORT)
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

bool TrossenArmHardwareInterface::parse_interface(
    const std::string & iface,
    std::string & joint_name,
    std::string & type)
{
  // Interface names are in the format: `<joint_name>/<interface_type>`
  auto slash_pos = iface.rfind('/');
  if (slash_pos == std::string::npos) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid interface format '%s'. Expected '<joint_name>/<type>'.",
      iface.c_str());
    return false;
  }
  joint_name = iface.substr(0, slash_pos);
  type = iface.substr(slash_pos + 1);
  return true;
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
TrossenArmHardwareInterface::interface_types_from_list(const std::vector<std::string> & ifaces) {
  std::set<std::string> types;
  for (const auto & iface : ifaces) {
    auto slash_pos = iface.rfind('/');
    std::string type = (slash_pos == std::string::npos) ? iface : iface.substr(slash_pos + 1);
    types.insert(type);
  }
  return types;
};

bool TrossenArmHardwareInterface::validate_command_interface(
    const std::string & iface,
    std::string & joint_name,
    std::string & type)
{
  // Validate strucutre of interface
  if (!parse_interface(iface, joint_name, type)) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid interface name '%s' requested.",
      iface.c_str());
    return false;
  }
  // Validate type
  if (type != HW_IF_POSITION && type != HW_IF_VELOCITY && type != HW_IF_EFFORT) {
    RCLCPP_ERROR(
      get_logger(),
      "Unsupported command interface '%s' requested.",
      type.c_str());
    return false;
  }
  // Validate joint name
  auto it = joint_name_to_index_.find(joint_name);
  if (it == joint_name_to_index_.end()) {
    RCLCPP_ERROR(
      get_logger(),
      "Unknown joint name '%s' in interface '%s'.",
      joint_name.c_str(),
      iface.c_str());
    return false;
  }
  return true;
}

}  // namespace trossen_arm_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_hardware::TrossenArmHardwareInterface,
  hardware_interface::SystemInterface)
