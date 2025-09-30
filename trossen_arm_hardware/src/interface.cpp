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
  joint_torque_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Check that all joints have the same command interface
  if (info_.joints.size() > 0) {
    const std::string& expected_interface = info_.joints[0].command_interfaces.at(INDEX_COMMAND_INTERFACE_).name;

    bool all_same = std::all_of(
      info_.joints.begin(),
      info_.joints.end(),
      [&](const auto& joint) {
        return joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_).name == expected_interface;
      }
    );

    if (!all_same) {
      RCLCPP_ERROR(
        get_logger(),
        "Not all joints have the same command interface type.");
      return CallbackReturn::ERROR;
    }

    command_interface_ = expected_interface;
  }

  for (const auto & joint : info_.joints) {
    // Only a single command interface per joint is supported for now
    if (joint.command_interfaces.size() != COUNT_COMMAND_INTERFACES_) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu command interfaces found. %zu expected.",
        joint.name.c_str(),
        joint.command_interfaces.size(),
        COUNT_COMMAND_INTERFACES_);
      return CallbackReturn::ERROR;
    }

    // Only position or effort command interface is expected
    if (joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_).name != HW_IF_POSITION &&
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found. '%s' or '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(INDEX_COMMAND_INTERFACE_).name.c_str(),
        HW_IF_POSITION,
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
    if (command_interface_ == HW_IF_POSITION) {
        // Position command interfaces
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            info_.joints[i].name,
            HW_IF_POSITION,
            &joint_position_commands_[i]));
    }
    if (command_interface_ == HW_IF_EFFORT) {
        // Effort command interfaces
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            info_.joints[i].name,
            HW_IF_EFFORT,
            &joint_torque_commands_[i]));
    }
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

  if (command_interface_ == HW_IF_POSITION){
    arm_driver_->set_all_modes(trossen_arm::Mode::position);
  }

  if (command_interface_ == HW_IF_EFFORT) {
    arm_driver_->set_all_modes(trossen_arm::Mode::external_effort);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto modes = arm_driver_->get_modes();

  // Validate position mode
  if (std::any_of(
      modes.begin(), modes.end(),
      [](trossen_arm::Mode mode) {return (mode != trossen_arm::Mode::position && mode != trossen_arm::Mode::external_effort);}))
  {
    std::string msg_modes = "Joint modes are not all position or external_effort. Modes are: ";
    for (auto mode : modes) {
      msg_modes += std::to_string(static_cast<int8_t>(mode)) + " ";
    }
    RCLCPP_ERROR(get_logger(), "%s", msg_modes.c_str());
    return CallbackReturn::ERROR;
  }

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
    std::fill(joint_torque_commands_.begin(), joint_torque_commands_.end(), 0);
    first_update_ = false;
  }

  // Validate that we are not sending NaN or INF values to the driver
  if (command_interface_ == HW_IF_POSITION && std::any_of(
      joint_position_commands_.begin(), joint_position_commands_.end(),
      [this](double position) {
        return std::isnan(position) || std::isinf(position);
      }))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Position commands to the joints contain NaN or INF values.");
    return return_type::ERROR;
  }
  if (command_interface_ == HW_IF_EFFORT && std::any_of(
      joint_torque_commands_.begin(), joint_torque_commands_.end(),
      [this](double torque) {
        return std::isnan(torque) || std::isinf(torque);
      }))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Torque commands to the joints contain NaN or INF values.");
    return return_type::ERROR;
  }

  if (command_interface_ == HW_IF_POSITION) {
     arm_driver_->set_all_positions(joint_position_commands_, 0.0, false);
  }
  if (command_interface_ == HW_IF_EFFORT) {
     arm_driver_->set_all_external_efforts(joint_torque_commands_, 0.0, false);
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

}  // namespace trossen_arm_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_hardware::TrossenArmHardwareInterface,
  hardware_interface::SystemInterface)
