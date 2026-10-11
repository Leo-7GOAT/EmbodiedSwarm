#include <gtest/gtest.h>
#include <limits>
#include "embodied_swarm_hardware/embodied_swarm_system.hpp"
using embodied_swarm_hardware::EmbodiedSwarmSystem;
hardware_interface::HardwareInfo config() {
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["backend"]="mock";
  for(int i=0;i<7;++i) {
    hardware_interface::ComponentInfo joint;
    joint.name="joint"+std::to_string(i);
    joint.parameters={{"min","-2"},{"max","2"},{"initial","0"}};
    hardware_interface::InterfaceInfo command; command.name="position";
    joint.command_interfaces.push_back(command);
    info.joints.push_back(joint);
  }
  return info;
}
TEST(Hardware, RejectsMalformedConfig) {
  EmbodiedSwarmSystem wrong_count; auto info=config(); info.joints.pop_back();
  EXPECT_EQ(wrong_count.on_init(info),EmbodiedSwarmSystem::CallbackReturn::ERROR);
  EmbodiedSwarmSystem nan; info=config(); info.joints[0].parameters["initial"]="nan";
  EXPECT_EQ(nan.on_init(info),EmbodiedSwarmSystem::CallbackReturn::ERROR);
  EmbodiedSwarmSystem bounds; info=config(); info.joints[0].parameters["min"]="4";
  EXPECT_EQ(bounds.on_init(info),EmbodiedSwarmSystem::CallbackReturn::ERROR);
  EmbodiedSwarmSystem missing; info=config(); info.joints[0].parameters.erase("max");
  EXPECT_EQ(missing.on_init(info),EmbodiedSwarmSystem::CallbackReturn::ERROR);
  EmbodiedSwarmSystem backend; info=config(); info.hardware_parameters["backend"]="unvalidated_hardware";
  EXPECT_EQ(backend.on_init(info),EmbodiedSwarmSystem::CallbackReturn::ERROR);
}
TEST(Hardware, MockRateAndInvalidCommandLatch) {
  int argc=0; char **argv=nullptr; rclcpp::init(argc,argv);
  {
    EmbodiedSwarmSystem system;
    ASSERT_EQ(system.on_init(config()),EmbodiedSwarmSystem::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),EmbodiedSwarmSystem::CallbackReturn::SUCCESS);
    auto command=system.export_command_interfaces(); auto state=system.export_state_interfaces();
    ASSERT_EQ(command.size(),7u); ASSERT_EQ(state.size(),14u);
    command[0].set_value(1.0);
    auto stamp=rclcpp::Clock().now(); auto dt=rclcpp::Duration::from_seconds(.01);
    system.write(stamp,dt); system.read(stamp,dt);
    EXPECT_NEAR(state[0].get_value(),.02,1e-12);
    EXPECT_NEAR(state[1].get_value(),2.,1e-12);
    command[0].set_value(std::numeric_limits<double>::quiet_NaN());
    system.write(stamp,dt); system.read(stamp,dt);
    EXPECT_NEAR(state[0].get_value(),.02,1e-12);
    EXPECT_EQ(state[1].get_value(),0.);
    command[0].set_value(.5); system.write(stamp,dt); system.read(stamp,dt);
    EXPECT_NEAR(state[0].get_value(),.02,1e-12);
  }
  rclcpp::shutdown();
}
