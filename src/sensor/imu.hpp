#pragma once
#include "utils/rclcpp_parameter_node.hpp"
#include <Eigen/Dense>
#include <optional>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
namespace sleepy {
struct ImuSensor {
  std::string name;
  std::string frame_id;
  std::string topic;
  Eigen::Isometry3d sensor_in_frame = Eigen::Isometry3d::Identity();

  std::optional<Eigen::Isometry3d> frame_in_state;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
  double acc_scale;
  ImuSensor(const utils::ParamsNode &config) {
    topic = config.declare<std::string>("topic");
    frame_id = config.declare<std::string>("frame_id");
    sensor_in_frame.translation() =
        config.declare<Eigen::Vector3d>("sensor_in_frame_t");
    sensor_in_frame.linear() =
        config.declare<Eigen::Matrix3d>("sensor_in_frame_R");
    acc_scale = config.declare<double>("acc_scale");
  }
};
} // namespace sleepy
