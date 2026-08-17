#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace sleepy::utils::so3 {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d out;
    out << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return out;
}

inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& angle) {
    const double norm = angle.norm();
    if (norm < 1e-12) {
        return Eigen::Matrix3d::Identity() + hat(angle);
    }
    return Eigen::AngleAxisd(norm, angle / norm).toRotationMatrix();
}

inline Eigen::Matrix3d a_matrix(const Eigen::Vector3d& angle) {
    const double squared_norm = angle.squaredNorm();
    if (squared_norm < 1e-22) {
        return Eigen::Matrix3d::Identity();
    }
    const double norm = std::sqrt(squared_norm);
    const Eigen::Matrix3d angle_hat = hat(angle);
    return Eigen::Matrix3d::Identity() + (1.0 - std::cos(norm)) / squared_norm * angle_hat
        + (1.0 - std::sin(norm) / norm) / squared_norm * angle_hat * angle_hat;
}

} // namespace sleepy::utils::so3
