#include <gtest/gtest.h>
#include <limits>
#include <thread>
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

TEST(Hardware, PhysicsResetRequiresAcceptedFreshFeedback) {
  int argc=0; char **argv=nullptr; rclcpp::init(argc,argv);
  {
    EmbodiedSwarmSystem system; auto info=config(); info.hardware_parameters["backend"]="mujoco";
    ASSERT_EQ(system.on_init(info),EmbodiedSwarmSystem::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),EmbodiedSwarmSystem::CallbackReturn::SUCCESS);
    auto observer=std::make_shared<rclcpp::Node>("physics_reset_regression");
    auto publisher=observer->create_publisher<sensor_msgs::msg::JointState>("/mujoco/joint_states",10);
    auto client=observer->create_client<std_srvs::srv::Trigger>("/hardware/reset_safety");
    std::string status;
    auto subscription=observer->create_subscription<std_msgs::msg::String>("/hardware/safety_status",10,
      [&status](std_msgs::msg::String::SharedPtr m){status=m->data;});
    auto pump=[&]() {
      auto stamp=observer->now(); auto dt=rclcpp::Duration::from_seconds(.01);
      system.read(stamp,dt); system.write(stamp,dt); rclcpp::spin_some(observer);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    };
    auto wait_until=[&](auto condition) {
      auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
      while(!condition() && std::chrono::steady_clock::now()<deadline) pump();
      return condition();
    };
    ASSERT_TRUE(wait_until([&](){return client->service_is_ready() && publisher->get_subscription_count()>0 && status=="SOFTWARE_ESTOP_HOLD";}));
    auto reset=[&]() {
      auto future=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if(!wait_until([&](){return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;}))
        throw std::runtime_error("Hardware reset service response deadline exceeded");
      return future.get();
    };
    EXPECT_FALSE(reset()->success); // initialized state alone never permits reset
    sensor_msgs::msg::JointState message;
    for(const auto & joint:info.joints) {message.name.push_back(joint.name); message.position.push_back(.1); message.velocity.push_back(0.);}
    message.header.stamp=observer->now()-rclcpp::Duration::from_seconds(1.);
    publisher->publish(message);
    for(int i=0;i<20;++i) pump();
    EXPECT_FALSE(reset()->success); // real DDS delivery of a stale packet is rejected
    auto states=system.export_state_interfaces();
    message.header.stamp=observer->now(); publisher->publish(message);
    ASSERT_TRUE(wait_until([&](){return states[0].get_value()==.1;}));
    EXPECT_TRUE(reset()->success);
    ASSERT_TRUE(wait_until([&](){return status=="ACTIVE_mujoco";}));
    auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
    while(std::chrono::steady_clock::now()<deadline) pump();
    EXPECT_FALSE(reset()->success); // handshake does not weaken the 250 ms watchdog
    EXPECT_EQ(status,"SOFTWARE_ESTOP_HOLD");
  }
  rclcpp::shutdown();
}
