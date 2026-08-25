#include "embodied_swarm_hardware/embodied_swarm_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace embodied_swarm_hardware
{

EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return info_.joints.size() == 7 ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
EmbodiedSwarmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < joint_positions_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
  }
  interfaces.emplace_back("base_vx", hardware_interface::HW_IF_VELOCITY, &base_vx_state_);
  interfaces.emplace_back("base_wz", hardware_interface::HW_IF_VELOCITY, &base_wz_state_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
EmbodiedSwarmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < joint_position_commands_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &joint_position_commands_[i]);
  }
  interfaces.emplace_back("base_vx", hardware_interface::HW_IF_VELOCITY, &base_vx_command_);
  interfaces.emplace_back("base_wz", hardware_interface::HW_IF_VELOCITY, &base_wz_command_);
  return interfaces;
}

EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  joint_position_commands_ = joint_positions_;
  base_vx_command_ = 0.0;
  base_wz_command_ = 0.0;
  return CallbackReturn::SUCCESS;
}

EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  base_vx_command_ = 0.0;
  base_wz_command_ = 0.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EmbodiedSwarmSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  RobotStateBuffer state;
  if (!read_backend(state)) {
    return hardware_interface::return_type::ERROR;
  }
  joint_positions_ = state.joint_positions;
  joint_velocities_ = state.joint_velocities;
  base_vx_state_ = state.base_twist[0];
  base_wz_state_ = state.base_twist[2];
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EmbodiedSwarmSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  RobotCommandBuffer command;
  command.timestamp = time.seconds();
  command.base_twist = {base_vx_command_, 0.0, base_wz_command_};
  command.joint_position_targets = joint_position_commands_;
  return write_backend(command) ? hardware_interface::return_type::OK
                                : hardware_interface::return_type::ERROR;
}

bool EmbodiedSwarmSystem::read_backend(RobotStateBuffer &)
{
  // Skeleton gate: replace with a timestamped real transport before activation.
  return false;
}

bool EmbodiedSwarmSystem::write_backend(const RobotCommandBuffer &)
{
  // Skeleton gate: replace with real base and Franka command transport.
  return false;
}

}  // namespace embodied_swarm_hardware

PLUGINLIB_EXPORT_CLASS(
  embodied_swarm_hardware::EmbodiedSwarmSystem,
  hardware_interface::SystemInterface)
