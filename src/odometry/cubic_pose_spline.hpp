#pragma once

#include "utils/so3.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>

namespace sleepy::spline {

struct CubicBasis {
    std::array<double, 4> value {};
    std::array<double, 4> first {};
    std::array<double, 4> second {};

    static CubicBasis evaluate(double u) {
        const double t = std::clamp(u, 0.0, 1.0);
        const double t2 = t * t;
        const double t3 = t2 * t;
        return {
            {
                (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0,
                (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0,
                (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0,
                t3 / 6.0,
            },
            {
                -0.5 * (1.0 - t) * (1.0 - t),
                1.5 * t2 - 2.0 * t,
                -1.5 * t2 + t + 0.5,
                0.5 * t2,
            },
            {
                1.0 - t,
                3.0 * t - 2.0,
                1.0 - 3.0 * t,
                t,
            },
        };
    }
};

struct PoseEvaluation {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    Eigen::Vector3d tangent_first = Eigen::Vector3d::Zero();
    Eigen::Vector3d tangent_second = Eigen::Vector3d::Zero();
    CubicBasis basis;
};

class CubicPoseSpline {
public:
    using Controls = std::array<Eigen::Vector3d, 4>;

    CubicPoseSpline(
        double segment_start,
        double knot_dt,
        const Controls& position_controls,
        const Controls& rotation_controls,
        const Eigen::Matrix3d& rotation_anchor
    ):
        segment_start_(segment_start),
        knot_dt_(knot_dt),
        position_controls_(position_controls),
        rotation_controls_(rotation_controls),
        rotation_anchor_(rotation_anchor) {}

    [[nodiscard]] PoseEvaluation evaluate(double timestamp) const {
        const double u = (timestamp - segment_start_) / knot_dt_;
        const CubicBasis basis = CubicBasis::evaluate(u);
        PoseEvaluation output;
        output.basis = basis;
        for (std::size_t i = 0; i < 4; ++i) {
            output.position += basis.value[i] * position_controls_[i];
            output.velocity += basis.first[i] * position_controls_[i] / knot_dt_;
            output.acceleration += basis.second[i] * position_controls_[i] / (knot_dt_ * knot_dt_);
            output.tangent += basis.value[i] * rotation_controls_[i];
            output.tangent_first += basis.first[i] * rotation_controls_[i] / knot_dt_;
            output.tangent_second +=
                basis.second[i] * rotation_controls_[i] / (knot_dt_ * knot_dt_);
        }

        output.rotation = rotation_anchor_ * utils::so3::exp_so3(output.tangent);
        const Eigen::Matrix3d right_jacobian = utils::so3::a_matrix(-output.tangent);
        output.omega = right_jacobian * output.tangent_first;
        // The derivative of the SO(3) right Jacobian is second order in the
        // local tangent. This approximation is stable for the small per-knot
        // increments expected by the recursive filter.
        output.alpha = right_jacobian * output.tangent_second;
        return output;
    }

private:
    double segment_start_;
    double knot_dt_;
    const Controls& position_controls_;
    const Controls& rotation_controls_;
    const Eigen::Matrix3d& rotation_anchor_;
};

} // namespace sleepy::spline
