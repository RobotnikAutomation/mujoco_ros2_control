// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>

#include <mujoco/mujoco.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

namespace mujoco_ros2_control_plugins
{

class ImuPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  int sensor_address(const mjModel* model, const std::string& name, int expected_dimension) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("mujoco_imu_plugin") };
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> publisher_;
  sensor_msgs::msg::Imu message_;
  rclcpp::Duration publish_period_{ 0, 10000000 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };
  int quat_address_{ -1 };
  int gyro_address_{ -1 };
  int accel_address_{ -1 };
};

}  // namespace mujoco_ros2_control_plugins
