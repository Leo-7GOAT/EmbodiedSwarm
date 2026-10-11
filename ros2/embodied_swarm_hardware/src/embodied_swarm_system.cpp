#include "embodied_swarm_hardware/embodied_swarm_system.hpp"
#include <algorithm>
#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace embodied_swarm_hardware {
EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS || info.joints.size() != 7) return CallbackReturn::ERROR;
  auto p = info.hardware_parameters.find("backend");
  if (p != info.hardware_parameters.end()) backend_ = p->second;
  if (backend_ != "mock" && backend_ != "mujoco") return CallbackReturn::ERROR;
  try { for (size_t i=0; i<7; ++i) {
    if (info.joints[i].command_interfaces.size()!=1 || info.joints[i].command_interfaces[0].name!="position") return CallbackReturn::ERROR;
    auto & params = info.joints[i].parameters;
    lower_[i] = std::stod(params.at("min")); upper_[i] = std::stod(params.at("max"));
    positions_[i] = std::stod(params.at("initial")); commands_[i] = positions_[i];
    if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || !std::isfinite(positions_[i]) || lower_[i]>=upper_[i] || positions_[i]<lower_[i] || positions_[i]>upper_[i]) return CallbackReturn::ERROR;
  } } catch (const std::exception &) { return CallbackReturn::ERROR; }
  node_ = std::make_shared<rclcpp::Node>("embodied_hardware_transport");
  pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/mujoco/joint_commands", 10);
  status_ = node_->create_publisher<std_msgs::msg::String>("/hardware/safety_status", 10);
  period_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/hardware/control_period", 100);
  sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/mujoco/joint_states", 10,
    [this](sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->position.size()!=msg->name.size() || msg->velocity.size()!=msg->name.size()) return;
      const double age = (node_->now()-rclcpp::Time(msg->header.stamp)).seconds();
      if (age < -0.05 || age > timeout_) return;
      std::array<double,7> q{}, v{};
      for (size_t i=0;i<7;++i) {
        auto it=std::find(msg->name.begin(),msg->name.end(),info_.joints[i].name);
        if (it==msg->name.end()) return;
        auto j=static_cast<size_t>(it-msg->name.begin()); q[i]=msg->position[j]; v[i]=msg->velocity[j];
        if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || q[i]<lower_[i]-0.05 || q[i]>upper_[i]+0.05) return;
      }
      positions_=q; velocities_=v; received_=true; last_state_=std::chrono::steady_clock::now();
    });
  stop_service_=node_->create_service<std_srvs::srv::SetBool>("/hardware/software_estop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (req->data) { stopped_=true; res->success=true; res->message="Software estop latched; use reset after resolving fault"; }
    else { res->success=false; res->message="Use /hardware/reset_safety"; }
  });
  reset_service_=node_->create_service<std_srvs::srv::Trigger>("/hardware/reset_safety", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success=backend_=="mock" || (received_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-last_state_).count()<timeout_);
    if (res->success) { commands_=positions_; stopped_=false; }
    res->message=res->success ? "Reset to measured hold position" : "Fresh state required";
  });
  return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> EmbodiedSwarmSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for(size_t i=0;i<7;++i) {out.emplace_back(info_.joints[i].name,"position",&positions_[i]); out.emplace_back(info_.joints[i].name,"velocity",&velocities_[i]);} return out;
}
std::vector<hardware_interface::CommandInterface> EmbodiedSwarmSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out; for(size_t i=0;i<7;++i) out.emplace_back(info_.joints[i].name,"position",&commands_[i]); return out;
}
EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_activate(const rclcpp_lifecycle::State &) {
  commands_=positions_; last_state_=std::chrono::steady_clock::now();
  // The initialized pose is not feedback. Require an explicit reset after a
  // fresh physics packet, even when DDS discovery outlasts controller startup.
  if (backend_=="mujoco") { received_=false; stopped_=true; }
  return CallbackReturn::SUCCESS;
}
EmbodiedSwarmSystem::CallbackReturn EmbodiedSwarmSystem::on_deactivate(const rclcpp_lifecycle::State &) {commands_=positions_; stopped_=true; return CallbackReturn::SUCCESS;}
hardware_interface::return_type EmbodiedSwarmSystem::read(const rclcpp::Time &, const rclcpp::Duration & period) {
  rclcpp::spin_some(node_);
  std_msgs::msg::Float64 timing; timing.data=period.seconds(); period_pub_->publish(timing);
  if(backend_=="mock") for(size_t i=0;i<7;++i) {
    const double dt=std::clamp(period.seconds(),1e-6,0.1);
    const double delta=stopped_?0.0:std::clamp(commands_[i]-positions_[i],-rate_limit_*dt,rate_limit_*dt);
    positions_[i]+=delta; velocities_[i]=delta/dt;
  }
  if(backend_=="mujoco" && std::chrono::duration<double>(std::chrono::steady_clock::now()-last_state_).count()>timeout_) stopped_=true;
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type EmbodiedSwarmSystem::write(const rclcpp::Time & time, const rclcpp::Duration &) {
  for(size_t i=0;i<7;++i) if(!std::isfinite(commands_[i]) || commands_[i]<lower_[i] || commands_[i]>upper_[i]) stopped_=true;
  sensor_msgs::msg::JointState msg; msg.header.stamp=time;
  for(size_t i=0;i<7;++i) {msg.name.push_back(info_.joints[i].name); msg.position.push_back(stopped_?positions_[i]:commands_[i]);}
  pub_->publish(msg); std_msgs::msg::String status; status.data=stopped_?"SOFTWARE_ESTOP_HOLD":"ACTIVE_"+backend_; status_->publish(status);
  return hardware_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(embodied_swarm_hardware::EmbodiedSwarmSystem,hardware_interface::SystemInterface)
