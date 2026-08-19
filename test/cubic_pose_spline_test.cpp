#include "odometry/cubic_pose_spline.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <iostream>

namespace {

bool near(double lhs, double rhs, double tolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

bool near(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs, double tolerance) {
    return (lhs - rhs).norm() <= tolerance;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& matrix) {
    return {
        matrix(2, 1),
        matrix(0, 2),
        matrix(1, 0),
    };
}

} // namespace

int main() {
    for (double u: std::array<double, 5> { 0.0, 0.17, 0.5, 0.83, 1.0 }) {
        const auto basis = sleepy::spline::CubicBasis::evaluate(u);
        double value_sum = 0.0;
        double first_sum = 0.0;
        double second_sum = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
            value_sum += basis.value[i];
            first_sum += basis.first[i];
            second_sum += basis.second[i];
        }
        if (!near(value_sum, 1.0, 1e-12) || !near(first_sum, 0.0, 1e-12)
            || !near(second_sum, 0.0, 1e-12)) {
            std::cerr << "cubic basis partition test failed at u=" << u << '\n';
            return 1;
        }
    }

    sleepy::spline::CubicPoseSpline::Controls positions {
        Eigen::Vector3d(-0.2, 0.1, 0.0),
        Eigen::Vector3d(0.1, 0.4, -0.1),
        Eigen::Vector3d(0.8, 0.7, 0.3),
        Eigen::Vector3d(1.4, 1.0, 0.6),
    };
    sleepy::spline::CubicPoseSpline::Controls rotations {
        Eigen::Vector3d(0.01, -0.02, 0.00),
        Eigen::Vector3d(0.02, -0.01, 0.03),
        Eigen::Vector3d(0.04, 0.01, 0.05),
        Eigen::Vector3d(0.07, 0.03, 0.08),
    };
    const Eigen::Matrix3d anchor =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    constexpr double knot_dt = 0.01;
    constexpr double timestamp = 3.004;
    constexpr double epsilon = 1e-6;
    const sleepy::spline::CubicPoseSpline pose_spline(3.0, knot_dt, positions, rotations, anchor);
    const auto center = pose_spline.evaluate(timestamp);
    const auto before = pose_spline.evaluate(timestamp - epsilon);
    const auto after = pose_spline.evaluate(timestamp + epsilon);
    const Eigen::Vector3d velocity_fd = (after.position - before.position) / (2.0 * epsilon);
    const Eigen::Vector3d acceleration_fd = (after.velocity - before.velocity) / (2.0 * epsilon);
    if (!near(center.velocity, velocity_fd, 1e-5)
        || !near(center.acceleration, acceleration_fd, 1e-3)) {
        std::cerr << "translation derivative test failed\n";
        return 2;
    }

    const Eigen::Matrix3d rotation_derivative =
        (after.rotation - before.rotation) / (2.0 * epsilon);
    const Eigen::Vector3d omega_fd = vee(center.rotation.transpose() * rotation_derivative);
    if (!near(center.omega, omega_fd, 1e-4)) {
        std::cerr << "rotation derivative test failed: " << center.omega.transpose() << " vs "
                  << omega_fd.transpose() << '\n';
        return 3;
    }

    sleepy::spline::CubicPoseSpline::Controls static_controls;
    for (auto& control: static_controls) {
        control = Eigen::Vector3d(1.0, 2.0, 3.0);
    }
    sleepy::spline::CubicPoseSpline::Controls zero_rotations;
    for (auto& control: zero_rotations) {
        control.setZero();
    }
    const sleepy::spline::CubicPoseSpline
        static_spline(0.0, knot_dt, static_controls, zero_rotations, Eigen::Matrix3d::Identity());
    const auto static_evaluation = static_spline.evaluate(0.005);
    if (!near(static_evaluation.position, Eigen::Vector3d(1.0, 2.0, 3.0), 1e-12)
        || static_evaluation.velocity.norm() > 1e-12
        || static_evaluation.acceleration.norm() > 1e-10 || static_evaluation.omega.norm() > 1e-12
        || static_evaluation.alpha.norm() > 1e-10)
    {
        std::cerr << "static spline test failed\n";
        return 4;
    }
    return 0;
}
