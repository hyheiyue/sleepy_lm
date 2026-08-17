#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
namespace sleepy {
struct EstimationState {
    double timestamp = 0.0;

    // T_odom_state: state-frame coordinates to odometry-frame coordinates.
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    // The state origin velocity and gravity are expressed in odometry.
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();

    // Angular velocity and specific force at the state origin are expressed
    // in the state frame.
    Eigen::Vector3d omg = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
};
} // namespace sleepy
