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
struct ImuState {
    struct InitializationSample {
        double timestamp = 0.0;
        Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    };

    // T_state_imu maps IMU-frame coordinates into the state frame.
    Eigen::Isometry3d imu_in_state = Eigen::Isometry3d::Identity();

    // Biases remain in this IMU's own measurement frame.
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();

    Eigen::Vector3d mean_specific_force_at_state_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_omega_state = Eigen::Vector3d::Zero();
    std::vector<InitializationSample> initialization_samples;
    double first_timestamp = 0.0;
    double last_timestamp = 0.0;
    double last_motion_timestamp = 0.0;
    bool has_first_sample = false;
    bool has_last_motion = false;
    bool initialized = false;
};
} // namespace sleepy
