#ifndef EMBODIED_SWARM_HARDWARE__EMBODIED_SWARM_SYSTEM_HPP_
#define EMBODIED_SWARM_HARDWARE__EMBODIED_SWARM_SYSTEM_HPP_

#include <array>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace embodied_swarm_hardware
{

struct RobotStateBuffer
{
  double timestamp{0.0};
  std::array<double, 3> base_pose{};
  std::array<double, 3> base_twist{};
  std::array<double, 7> joint_positions{};
  std::array<double, 7> joint_velocities{};
  double gripper_width{0.0};
};

struct RobotCommandBuffer
{
  double timestamp{0.0};
  std::array<double, 3> base_twist{};
  std::array<double, 7> joint_position_targets{};
};

class EmbodiedSwarmSystem final : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_backend(RobotStateBuffer & state);
  bool write_backend(const RobotCommandBuffer & command);

  std::array<double, 7> joint_positions_{};
  std::array<double, 7> joint_velocities_{};
  std::array<double, 7> joint_position_commands_{};
  double base_vx_state_{0.0};
  double base_wz_state_{0.0};
  double base_vx_command_{0.0};
  double base_wz_command_{0.0};
};

}  // namespace embodied_swarm_hardware

#endif  // EMBODIED_SWARM_HARDWARE__EMBODIED_SWARM_SYSTEM_HPP_
