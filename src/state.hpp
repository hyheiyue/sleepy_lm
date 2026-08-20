#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
namespace sleepy {
struct EstimationState {
    // Timestamp of this state, in seconds.
    double timestamp = 0.0;

    // T_odom_state: maps coordinates from the state frame into odometry.
    Eigen::Isometry3d state_in_odom = Eigen::Isometry3d::Identity();

    // Position, velocity and gravity are expressed in odometry; angular
    // velocity and specific force below remain expressed in the state frame,
    // matching the ESKF convention used by small_point_lio.
    Eigen::Vector3d velocity_odom = Eigen::Vector3d::Zero();

    // Angular velocity of the state frame, expressed in state coordinates.
    Eigen::Vector3d omg = Eigen::Vector3d::Zero();

    // Specific force at the state origin, excluding gravity, in state
    // coordinates and m/s^2.
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    // Gravitational acceleration, in m/s^2.
    Eigen::Vector3d gravity_odom = Eigen::Vector3d::Zero();
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

    // Mean specific force at the IMU lever arm projected to the state origin,
    // expressed in the state frame.
    Eigen::Vector3d mean_specific_force_at_state_origin = Eigen::Vector3d::Zero();
    // Intermediate IMU-origin angular velocity expressed in the state frame,
    // used to estimate angular acceleration for lever-arm compensation.
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
