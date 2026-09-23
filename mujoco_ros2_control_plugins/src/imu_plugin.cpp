// Copyright 2026 Robotnik Automation S.L.
// Licensed under the Apache License, Version 2.0.

#include "imu_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

int ImuPlugin::sensor_address(const mjModel* model, const std::string& name, int expected_dimension) const
{
  const int id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
  if (id < 0 || model->sensor_dim[id] != expected_dimension)
  {
    RCLCPP_ERROR(logger_, "MuJoCo sensor '%s' is missing or has the wrong dimension (expected %d).", name.c_str(),
                 expected_dimension);
    return -1;
  }
  return model->sensor_adr[id];
}

bool ImuPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node_->get_sub_namespace());
  const std::string prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  auto declare_string = [&](const std::string& key, const std::string& default_value) {
    const std::string full = prefix + key;
    if (!node_->has_parameter(full))
    {
      node_->declare_parameter(full, default_value);
    }
    return node_->get_parameter(full).as_string();
  };
  auto declare_double = [&](const std::string& key, double default_value) {
    const std::string full = prefix + key;
    if (!node_->has_parameter(full))
    {
      node_->declare_parameter(full, default_value);
    }
    return node_->get_parameter(full).as_double();
  };

  const std::string sensor_name = declare_string("sensor_name", "");
  const std::string frame_name = declare_string("frame_name", sensor_name + "_frame");
  const std::string topic = declare_string("topic", "imu/data");
  const double publish_rate = declare_double("publish_rate", 100.0);
  if (sensor_name.empty() || publish_rate <= 0.0)
  {
    RCLCPP_ERROR(logger_, "sensor_name must be non-empty and publish_rate must be positive.");
    return false;
  }

  quat_address_ = sensor_address(model, sensor_name + "_quat", 4);
  gyro_address_ = sensor_address(model, sensor_name + "_gyro", 3);
  accel_address_ = sensor_address(model, sensor_name + "_accel", 3);
  if (quat_address_ < 0 || gyro_address_ < 0 || accel_address_ < 0)
  {
    return false;
  }

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  last_publish_time_ = node_->get_clock()->now();
  message_.header.frame_id = frame_name;
  publisher_raw_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      topic, rclcpp::SensorDataQoS().keep_last(5));
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(publisher_raw_);
  RCLCPP_INFO(logger_, "Publishing MuJoCo IMU '%s' on '%s' at %.1f Hz.", sensor_name.c_str(),
              publisher_raw_->get_topic_name(), publish_rate);
  return true;
}

void ImuPlugin::update(const mjModel* /*model*/, mjData* data)
{
  const rclcpp::Time now = node_->get_clock()->now();
  if (now - last_publish_time_ < publish_period_)
  {
    return;
  }
  last_publish_time_ = now;
  message_.header.stamp = now;
  // MuJoCo quaternions are ordered w, x, y, z.
  message_.orientation.w = data->sensordata[quat_address_];
  message_.orientation.x = data->sensordata[quat_address_ + 1];
  message_.orientation.y = data->sensordata[quat_address_ + 2];
  message_.orientation.z = data->sensordata[quat_address_ + 3];
  message_.angular_velocity.x = data->sensordata[gyro_address_];
  message_.angular_velocity.y = data->sensordata[gyro_address_ + 1];
  message_.angular_velocity.z = data->sensordata[gyro_address_ + 2];
  message_.linear_acceleration.x = data->sensordata[accel_address_];
  message_.linear_acceleration.y = data->sensordata[accel_address_ + 1];
  message_.linear_acceleration.z = data->sensordata[accel_address_ + 2];
#if REALTIME_TOOLS_VERSION_MAJOR > 2
  publisher_->try_publish(message_);
#else
  publisher_->tryPublish(message_);
#endif
}

void ImuPlugin::cleanup()
{
  publisher_.reset();
  publisher_raw_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ImuPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
