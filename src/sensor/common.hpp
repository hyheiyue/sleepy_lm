#pragma once
#include <Eigen/Dense>
#include <cstddef>
namespace sleepy::common {
struct SensorTag {
  std::size_t id = 0;
};
struct Point {
  double timestamp = 0.0; // 点级时间戳，单位：s
  Eigen::Vector3f position = Eigen::Vector3f::Zero(); // 点坐标，单位：m
  int count = 1;
  SensorTag sensor;
};
struct ImuMsg {
  double timestamp = 0.0; // 绝对参考时间戳，单位：s
  Eigen::Vector3d linear_acceleration =
      Eigen::Vector3d::Zero(); // 线加速度，单位按 acc_norm 缩放
  Eigen::Vector3d angular_velocity =
      Eigen::Vector3d::Zero(); // 角速度，单位：rad/s
  SensorTag sensor;
};
} // namespace sleepy::common
