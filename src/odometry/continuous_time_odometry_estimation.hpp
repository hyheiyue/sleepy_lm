#pragma once
#include "gtsam_points/types/point_cloud_cpu.hpp"
#include "gtsam_points/util/continuous_trajectory.hpp"
#include "sensor/common.hpp"
#include "sensor/imu.hpp"
#include "sensor/point_cloud.hpp"
#include "state.hpp"
#include "utils/logger.hpp"
#include "utils/rclcpp_parameter_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>
namespace sleepy {
class ContinuousTimeOdometryEstimation {
public:
    struct Params {
        bool align_gravity_with_z_axiz;
        double initialization_window;
        int initialization_min_samples;
        double gravity_norm;
        void load(const utils::ParamsNode& config) {
            align_gravity_with_z_axiz = config.declare<bool>("align_gravity_with_z_axiz");
            initialization_window = std::max(0.0, config.declare<double>("initialization_window"));
            initialization_min_samples =
                std::max(1, config.declare<int>("initialization_min_samples"));
            gravity_norm = config.declare<double>("gravity_norm");
        }
    };

    struct ImuState {
        struct InitializationSample {
            double timestamp = 0.0;
            Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
            Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        };

        // T_state_imu maps IMU-frame coordinates into the state frame.
        Eigen::Isometry3d imu_in_state = Eigen::Isometry3d::Identity();

        // Biases stay in this IMU's own measurement frame. They must not be
        // rotated into the state frame and must not be shared by other IMUs.
        Eigen::Vector3d ba = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg = Eigen::Vector3d::Zero();

        Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_specific_force_at_state_origin = Eigen::Vector3d::Zero();
        std::vector<InitializationSample> initialization_samples;
        double first_timestamp = 0.0;
        double last_timestamp = 0.0;
        std::size_t sample_count = 0;
        bool has_first_sample = false;
        bool initialized = false;
    };

    explicit ContinuousTimeOdometryEstimation(const utils::ParamsNode& config) {
        params_.load(config);

        state_.pose.setIdentity();
        state_.gravity.setZero();
    }

    [[nodiscard]] const EstimationState& state() const noexcept {
        return state_;
    }
    [[nodiscard]] const std::vector<ImuState>& imu_states() const noexcept {
        return imu_states_;
    }
    [[nodiscard]] bool initialized() const noexcept {
        return inited;
    }
    [[nodiscard]] bool map_initialized() const noexcept {
        return map_ && map_->size() != 0;
    }
    [[nodiscard]] gtsam_points::PointCloudCPU::ConstPtr map() const noexcept {
        return map_;
    }

    void add_point(const common::Point& point, const std::vector<PointCloudSensor>& sensors) {
        if (!inited || point.sensor.id >= sensors.size() || !std::isfinite(point.timestamp)
            || !point.position.allFinite() || point.timestamp < state_.timestamp)
        {
            return;
        }

        const auto& sensor = sensors[point.sensor.id];
        if (!sensor.frame_in_state) {
            return;
        }

        // T_odom_lidar maps LiDAR-frame coordinates into the odometry frame.
        const Eigen::Isometry3d T_odom_lidar =
            state_.pose * sensor.frame_in_state.value() * sensor.sensor_in_frame;
        const Eigen::Vector3d position_odom = T_odom_lidar * point.position.cast<double>();
        if (!position_odom.allFinite()) {
            return;
        }
        const Eigen::Vector4d point_odom = position_odom.homogeneous();

        if (!map_) {
            map_ = std::make_shared<gtsam_points::PointCloudCPU>();
        }

        map_->add_points(&point_odom, 1);
    }

    void add_imu(const common::ImuMsg& imu, const std::vector<ImuSensor>& sensors) {
        if (imu.sensor.id >= sensors.size() || !imu.linear_acceleration.allFinite()
            || !imu.angular_velocity.allFinite() || !std::isfinite(imu.timestamp))
        {
            return;
        }

        if (imu_states_.size() < sensors.size()) {
            imu_states_.resize(sensors.size());
        }

        const auto& sensor = sensors[imu.sensor.id];
        if (!sensor.frame_in_state) {
            return;
        }

        auto& imu_state = imu_states_[imu.sensor.id];
        imu_state.imu_in_state = sensor.frame_in_state.value() * sensor.sensor_in_frame;

        const auto initialize_imu = [](ImuState& state) {
            if (state.sample_count == 0) {
                return;
            }

            state.bg = state.gyro_sum / static_cast<double>(state.sample_count);

            // A stationary accelerometer only observes gravity plus ba. Without a
            // known attitude or an external acceleration-bias calibration, ba is
            // not separately observable, so keep its per-IMU initial value at zero.
            state.ba.setZero();

            const Eigen::Matrix3d& R_state_imu = state.imu_in_state.linear();
            const Eigen::Vector3d& r_state_imu = state.imu_in_state.translation();
            Eigen::Vector3d corrected_specific_force_sum = Eigen::Vector3d::Zero();

            for (std::size_t i = 0; i < state.initialization_samples.size(); ++i) {
                const auto& sample = state.initialization_samples[i];
                const Eigen::Vector3d omega_state =
                    R_state_imu * (sample.angular_velocity - state.bg);

                Eigen::Vector3d alpha_state = Eigen::Vector3d::Zero();
                if (state.initialization_samples.size() >= 2) {
                    const std::size_t first = i == 0 ? 0 : i - 1;
                    const std::size_t last =
                        i + 1 < state.initialization_samples.size() ? i + 1 : i;
                    const double dt = state.initialization_samples[last].timestamp
                        - state.initialization_samples[first].timestamp;
                    if (dt != 0.0) {
                        const Eigen::Vector3d omega_first = R_state_imu
                            * (state.initialization_samples[first].angular_velocity - state.bg);
                        const Eigen::Vector3d omega_last = R_state_imu
                            * (state.initialization_samples[last].angular_velocity - state.bg);
                        alpha_state = (omega_last - omega_first) / dt;
                    }
                }

                const Eigen::Vector3d specific_force_at_imu =
                    R_state_imu * (sample.linear_acceleration - state.ba);
                const Eigen::Vector3d lever_arm_acceleration = alpha_state.cross(r_state_imu)
                    + omega_state.cross(omega_state.cross(r_state_imu));
                corrected_specific_force_sum += specific_force_at_imu - lever_arm_acceleration;
            }

            if (!state.initialization_samples.empty()) {
                state.mean_specific_force_at_state_origin = corrected_specific_force_sum
                    / static_cast<double>(state.initialization_samples.size());
            }
            state.initialization_samples.clear();
            state.initialized = true;
        };

        const auto initialize_state = [this](double timestamp) {
            const auto align_vector_to_z_axis = [](const Eigen::Vector3d& vector
                                                ) -> Eigen::Matrix3d {
                if (!vector.allFinite() || vector.squaredNorm() == 0.0) {
                    return Eigen::Matrix3d::Identity();
                }
                return Eigen::Quaterniond::FromTwoVectors(
                           vector.normalized(),
                           Eigen::Vector3d::UnitZ()
                )
                    .toRotationMatrix();
            };

            Eigen::Vector3d gravity_sum = Eigen::Vector3d::Zero();
            std::size_t initialized_imus = 0;

            for (const auto& state: imu_states_) {
                if (!state.initialized || state.sample_count == 0) {
                    continue;
                }

                const Eigen::Vector3d& mean_specific_force_state =
                    state.mean_specific_force_at_state_origin;
                if (!mean_specific_force_state.allFinite()
                    || mean_specific_force_state.squaredNorm() == 0.0) {
                    continue;
                }

                // Accelerometers measure specific force, so stationary gravity is the
                // opposite direction of the averaged accelerometer measurement.
                gravity_sum += -mean_specific_force_state.normalized();
                ++initialized_imus;
            }

            if (imu_states_.empty() || initialized_imus != imu_states_.size()
                || gravity_sum.squaredNorm() == 0.0)
            {
                return;
            }

            state_.timestamp = timestamp;
            state_.pose.setIdentity();
            state_.vel.setZero();
            state_.omg.setZero();
            state_.acc.setZero();
            state_.gravity = params_.gravity_norm * gravity_sum.normalized();

            // T_odom_state maps the state frame to odometry. When enabled, rotate
            // the initial odometry frame so +Z points along physical gravity.
            if (params_.align_gravity_with_z_axiz) {
                state_.pose.linear() = align_vector_to_z_axis(state_.gravity);
            }

            inited = true;
            utils::log_info(
                "continuous-time odometry initialized: {} IMU(s), timestamp "
                "{:.6f}, gravity [{:.4f}, {:.4f}, {:.4f}], "
                "align_gravity_with_z_axia={}",
                initialized_imus,
                state_.timestamp,
                state_.gravity.x(),
                state_.gravity.y(),
                state_.gravity.z(),
                params_.align_gravity_with_z_axiz
            );
        };

        // Initialization is performed independently for every IMU. This keeps
        // ba/bg in the coordinates in which the corresponding sensor reports
        // them, while only the measured gravity direction is transformed into
        // the state frame.
        if (!imu_state.initialized) {
            if (!imu_state.has_first_sample) {
                imu_state.first_timestamp = imu.timestamp;
                imu_state.has_first_sample = true;
            }

            if (imu.timestamp < imu_state.last_timestamp && imu_state.sample_count) {
                return;
            }

            imu_state.acc_sum += imu.linear_acceleration;
            imu_state.gyro_sum += imu.angular_velocity;
            imu_state.initialization_samples.push_back(ImuState::InitializationSample {
                .timestamp = imu.timestamp,
                .linear_acceleration = imu.linear_acceleration,
                .angular_velocity = imu.angular_velocity,
            });
            imu_state.last_timestamp = imu.timestamp;
            ++imu_state.sample_count;

            const bool enough_samples = imu_state.sample_count
                >= static_cast<std::size_t>(params_.initialization_min_samples);
            const bool enough_time =
                imu.timestamp - imu_state.first_timestamp >= params_.initialization_window;
            if (enough_samples && enough_time) {
                initialize_imu(imu_state);
            }
        }

        if (!inited) {
            initialize_state(imu.timestamp);
        }
    }

    Params params_;
    EstimationState state_;
    std::vector<ImuState> imu_states_;
    // Target map points are stored in the odometry frame for later GICP use.
    gtsam_points::PointCloudCPU::Ptr map_;
    bool inited = false;
};
} // namespace sleepy
