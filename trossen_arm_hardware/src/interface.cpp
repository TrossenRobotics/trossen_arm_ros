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

  // Get robot model, ip
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
    // TODO(lukeschmitt-tr): validate robot model
    // RCLCPP_FATAL(
    //   get_logger(),
    //   "Invalid 'robot_model' value specified: '%s'.", robot_model);
    return CallbackReturn::FAILURE;
  }

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
      DRIVER_IP_ADDRESS_DEFAULT.c_str());
    driver_ip_address_ = DRIVER_IP_ADDRESS_DEFAULT;
  }

  try {
    update_period_ = std::stof(info.hardware_parameters.at("update_period"));
    RCLCPP_INFO(
      get_logger(),
      "Parameter 'update_period' set to '%f'.",
      update_period_);
  } catch (const std::out_of_range & /*e*/) {
    RCLCPP_WARN(
      get_logger(),
      "Parameter 'update_period' not specified. Defaulting to '%f'.",
      update_period_);
  }

  joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  joint_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints) {
    // Only position command interfaces are supported for now
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Only position expected
    if (joint.command_interfaces.at(0).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' command interface found. '%s' expected",
        joint.name.c_str(),
        joint.command_interfaces.at(0).name.c_str(),
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Each joint has 3 state interfaces: position, velocity, effort (in that order)
    // Expect three state interfaces
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has %zu state interfaces found. 3 expected.",
        joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Position first
    if (joint.state_interfaces.at(0).name != HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(0).name.c_str(),
        HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Velocity second
    if (joint.state_interfaces.at(1).name != HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(1).name.c_str(),
        HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Effort third
    if (joint.state_interfaces.at(2).name != HW_IF_EFFORT) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' has '%s' state interface found. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(2).name.c_str(),
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
    // Position command interfaces
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        HW_IF_POSITION,
        &joint_position_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn
TrossenArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
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
      trossen_arm::StandardEndEffector::wxai_v0_base,
      driver_ip_address_.c_str(),
      true);  // TODO(lukeschmitt-tr): Make this configuration (or remove it)
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to configure TrossenArmDriver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "TrossenArmDriver configured with model %d, IP Address '%s', port '%d', timeout %dus.",
    static_cast<int>(robot_model_), driver_ip_address_.c_str(), driver_port_, driver_timeout_us_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TrossenArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  first_update_ = true;

  arm_driver_->set_all_modes(trossen_arm::Mode::position);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto modes = arm_driver_->get_modes();

  // Validate position mode
  if (std::any_of(
      modes.begin(), modes.end(),
      [](trossen_arm::Mode mode) {return mode != trossen_arm::Mode::position;}))
  {
    std::string msg_modes = "Joint modes are not all position. Modes are: ";
    for (auto mode : modes) {
      msg_modes += std::to_string(static_cast<int8_t>(mode)) + " ";
    }
    RCLCPP_ERROR(get_logger(), msg_modes.c_str());
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
  // Get joint positions
  auto positions = arm_driver_->get_positions();

  // Copy joint positions
  joint_positions_.clear();
  joint_positions_.reserve(positions.size());
  for (const auto & pos : positions) {
    joint_positions_.push_back(static_cast<double>(pos));
  }
  // joint_positions_.push_back(-joint_positions_.back());

  // Get joint velocities
  auto velocities = arm_driver_->get_velocities();

  // Copy joint velocities
  joint_velocities_.clear();
  joint_velocities_.reserve(velocities.size());
  for (const auto & velocity : velocities) {
    joint_velocities_.push_back(static_cast<double>(velocity));
  }
  // joint_velocities_.push_back(-joint_velocities_.back());

  // Get joint efforts
  auto efforts = arm_driver_->get_external_efforts();

  // Copy joint efforts
  joint_efforts_.clear();
  joint_efforts_.reserve(efforts.size());
  for (const auto & effort : efforts) {
    joint_efforts_.push_back(static_cast<double>(effort));
  }
  // joint_efforts_.push_back(-joint_efforts_.back());

  return return_type::OK;
}

return_type
TrossenArmHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // If first time writing to the hardware, set all commands to the current positions
  if (first_update_) {
    joint_position_commands_ = joint_positions_;
    first_update_ = false;
    RCLCPP_DEBUG(
      get_logger(),
      "First write update. Setting joint position commands to current positions.");
  }

  auto position_commands = std::vector<float>(joint_positions_.size(), 0.0);

  // Copy joint position commands
  for (size_t i = 0; i < joint_positions_.size(); i++) {
    position_commands[i] = static_cast<float>(joint_position_commands_[i]);
  }

  // Validate that we are not sending NaN or INF values to the driver
  if (std::any_of(
      position_commands.begin(), position_commands.end(),
      [this](float position) {
        return std::isnan(position) || std::isinf(position);
      }))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Commands to the joints contain NaN or INF values.");
    return return_type::ERROR;
  }

  // arm_driver_->set_all_positions(position_commands, update_period_, false);

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
  arm_driver_->cleanup();
  arm_driver_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace trossen_arm_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  trossen_arm_hardware::TrossenArmHardwareInterface,
  hardware_interface::SystemInterface)
