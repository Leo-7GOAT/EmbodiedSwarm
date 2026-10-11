cmake_minimum_required(VERSION 3.8)
project(embodied_swarm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/embodied_swarm_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs std_msgs std_srvs)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

pluginlib_export_plugin_description_file(
  hardware_interface embodied_swarm_hardware.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES embodied_swarm_hardware.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/acceptance.py scripts/mujoco_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs std_msgs std_srvs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge_initialization test/test_bridge_initialization.py TIMEOUT 60)
  ament_add_gtest(test_system test/test_system.cpp)
  set_tests_properties(test_system PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=75;ROS_LOCALHOST_ONLY=1")
  target_link_libraries(test_system ${PROJECT_NAME})
  ament_target_dependencies(test_system hardware_interface rclcpp rclcpp_lifecycle)
endif()
ament_package()
