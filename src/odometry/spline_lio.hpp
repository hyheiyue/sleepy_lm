#pragma once

#include "ivox.hpp"
#include "sensor/imu.hpp"
#include "sensor/point_cloud.hpp"
#include "state.hpp"
#include "utils/logger.hpp"
#include "utils/rclcpp_parameter_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

namespace sleepy {

class SplineLio {
private:
    static constexpr int SPLINE_CP_DIM = 6;
    static constexpr int SPLINE_CP_NUM = 4;
    static constexpr int POSITION_OFFSET = 0;
    static constexpr int ROTATION_OFFSET = 3;
    // 4 个控制点：4 * 6 = 24 维，索引范围 [0, 24)
    static constexpr int SPLINE_STATE_DIM = SPLINE_CP_DIM * SPLINE_CP_NUM;

    // 重力从第 24 维开始，占用 [24, 27)
    static constexpr int GRAVITY_INDEX = SPLINE_STATE_DIM;
    static constexpr int BASE_STATE_DIM = SPLINE_STATE_DIM + 3;

    static constexpr int IMU_ERROR_DIM = 6;
    static constexpr std::size_t MAX_IMUS = 4;
    static constexpr int MAX_ERROR_STATE_DIM =
        BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * MAX_IMUS);

    using Covariance = Eigen::Matrix<double, MAX_ERROR_STATE_DIM, MAX_ERROR_STATE_DIM>;
    using ErrorState = Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 1>;
    static int spline_cp_error_index(std::size_t cp_id, int offset = 0) {
        return static_cast<int>(cp_id) * SPLINE_CP_DIM + offset;
    }
    static int imu_error_index(std::size_t imu_id, int offset = 0) {
        return BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * imu_id) + offset;
    }
    static constexpr int BA_OFFSET = 0;
    static constexpr int BG_OFFSET = 3;

    [[nodiscard]] int active_error_state_dim() const {
        return BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * imu_states_.size());
    }
    struct Params {
        double gravity_norm = 9.80665;
        bool align_gravity_with_z_axis = true;
        int initialization_min_samples = 200;
        double initialization_window = 1.0;
        int initial_map_size = 1000;
        double map_resolution = 0.1;
        int map_capacity = 1000000;
        double batch_interval = 0.001;

        void load(const utils::ParamsNode& config) {
            gravity_norm = config.declare<double>("gravity_norm");
            align_gravity_with_z_axis = config.declare<bool>("align_gravity_with_z_axis");
            initialization_min_samples = config.declare<int>("initialization_min_samples");
            initialization_window = config.declare<double>("initialization_window");
            initial_map_size = config.declare<int>("initial_map_size");
            map_resolution = config.declare<double>("map_resolution");
            map_capacity = config.declare<int>("map_capacity");
            batch_interval = config.declare<double>("batch_interval");
        }
    };

    struct MeasurementBatch {
        double batch_start_timestamp = -1.0;
        double batch_end_timestamp = -1.0;
        std::vector<common::Point> point_batch;
        std::vector<common::ImuMsg> imu_batch;

        void reset() {
            batch_start_timestamp = -1.0;
            batch_end_timestamp = -1.0;
            point_batch.clear();
            imu_batch.clear();
        }
    };

    enum class InitStage { WaitingImu, BuildingMap, Finished };

public:
    explicit SplineLio(const utils::ParamsNode& config) {
        params_.load(config);
        ivox_ = std::make_shared<SmallIVox>(
            static_cast<float>(params_.map_resolution),
            static_cast<std::size_t>(params_.map_capacity)
        );
    }

    [[nodiscard]] const EstimationState& state() const noexcept {
        return state_;
    }

    [[nodiscard]] bool state_initialized() const noexcept {
        return init_stage_ != InitStage::WaitingImu;
    }

    void add_imu(const common::ImuMsg& imu, const std::vector<ImuSensor>& sensors) {
        if (imu.sensor.id >= sensors.size() || !std::isfinite(imu.timestamp)
            || !imu.linear_acceleration.allFinite() || !imu.angular_velocity.allFinite())
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
        if (!imu_state.initialized && !state_initialized()) {
            if (!imu_state.has_first_sample) {
                imu_state.first_timestamp = imu.timestamp;
                imu_state.has_first_sample = true;
            } else if (imu.timestamp < imu_state.last_timestamp) {
                return;
            }

            imu_state.initialization_samples.push_back(ImuState::InitializationSample {
                .timestamp = imu.timestamp,
                .linear_acceleration = imu.linear_acceleration,
                .angular_velocity = imu.angular_velocity,
            });
            imu_state.last_timestamp = imu.timestamp;

            const bool enough_samples = imu_state.initialization_samples.size()
                >= static_cast<std::size_t>(params_.initialization_min_samples);
            const bool enough_time =
                imu.timestamp - imu_state.first_timestamp >= params_.initialization_window;
            if (enough_samples && enough_time) {
                initialize_imu(imu_state);
            }
            if (!imu_states_.empty()
                && std::all_of(imu_states_.begin(), imu_states_.end(), [](const ImuState& state) {
                       return state.initialized;
                   }))
            {
                initialize_state(imu.timestamp);
            }
            return;
        }
        add_to_batch(imu, batch_.imu_batch);
    }

    void
    add_point(const common::Point& point_in_lidar, const std::vector<PointCloudSensor>& sensors) {
        if (point_in_lidar.sensor.id >= sensors.size() || !std::isfinite(point_in_lidar.timestamp)
            || !point_in_lidar.position.allFinite())
        {
            return;
        }

        const auto& sensor = sensors[point_in_lidar.sensor.id];
        if (!sensor.frame_in_state) {
            return;
        }
        const Eigen::Isometry3d lidar_in_state =
            sensor.frame_in_state.value() * sensor.sensor_in_frame;
        const Eigen::Vector3d point_state = lidar_in_state * point_in_lidar.position.cast<double>();

        if (!state_initialized()) {
            pending_initial_points_.push_back(point_state.cast<float>());
            return;
        }

        if (point_in_lidar.timestamp < state_.timestamp) {
            return;
        }

        if (init_stage_ == InitStage::BuildingMap) {
            initial_points_.push_back((state_.pose * point_state).cast<float>());
            if (initial_points_.size() >= static_cast<std::size_t>(params_.initial_map_size)) {
                for (const auto& point: initial_points_) {
                    ivox_->add_point(point);
                }
                initial_points_.clear();
                init_stage_ = InitStage::Finished;
                utils::log_info("Spline-LIO map initialized");
            }
            return;
        }

        if (init_stage_ != InitStage::Finished) {
            return;
        }
        add_to_batch(point_in_lidar, batch_.point_batch);
    }

private:
    template<typename Message>
    void add_to_batch(const Message& message, std::vector<Message>& batch) {
        batch.push_back(message);
        if (batch_.batch_start_timestamp < 0.0) {
            batch_.batch_start_timestamp = message.timestamp;
        }
        batch_.batch_end_timestamp = message.timestamp;
        if (batch_.batch_end_timestamp - batch_.batch_start_timestamp >= params_.batch_interval) {
            update_batch();
        }
    }

    void update_batch() {
        batch_.reset();
    }

    void initialize_imu(ImuState& imu_state) {
        if (imu_state.initialization_samples.empty()) {
            return;
        }

        Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
        for (const auto& sample: imu_state.initialization_samples) {
            gyro_sum += sample.angular_velocity;
        }
        imu_state.bg = gyro_sum / static_cast<double>(imu_state.initialization_samples.size());
        imu_state.ba.setZero();

        const Eigen::Matrix3d R_state_imu = imu_state.imu_in_state.linear();
        const Eigen::Vector3d r_state_imu = imu_state.imu_in_state.translation();
        Eigen::Vector3d corrected_force_sum = Eigen::Vector3d::Zero();

        for (std::size_t i = 0; i < imu_state.initialization_samples.size(); ++i) {
            const auto& sample = imu_state.initialization_samples[i];
            const Eigen::Vector3d omega = R_state_imu * (sample.angular_velocity - imu_state.bg);
            Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
            if (imu_state.initialization_samples.size() > 1) {
                const std::size_t first = i == 0 ? i : i - 1;
                const std::size_t last =
                    i + 1 < imu_state.initialization_samples.size() ? i + 1 : i;
                const double dt = imu_state.initialization_samples[last].timestamp
                    - imu_state.initialization_samples[first].timestamp;
                if (dt > 0.0) {
                    const Eigen::Vector3d omega_first = R_state_imu
                        * (imu_state.initialization_samples[first].angular_velocity - imu_state.bg);
                    const Eigen::Vector3d omega_last = R_state_imu
                        * (imu_state.initialization_samples[last].angular_velocity - imu_state.bg);
                    alpha = (omega_last - omega_first) / dt;
                }
            }
            const Eigen::Vector3d lever =
                alpha.cross(r_state_imu) + omega.cross(omega.cross(r_state_imu));
            corrected_force_sum +=
                R_state_imu * (sample.linear_acceleration - imu_state.ba) - lever;
        }

        imu_state.mean_specific_force_at_state_origin =
            corrected_force_sum / static_cast<double>(imu_state.initialization_samples.size());
        imu_state.initialization_samples.clear();
        imu_state.initialized = true;
    }

    bool initialize_state(double timestamp) {
        Eigen::Vector3d gravity_state = Eigen::Vector3d::Zero();
        for (const auto& imu: imu_states_) {
            if (!imu.initialized || !imu.mean_specific_force_at_state_origin.allFinite()
                || imu.mean_specific_force_at_state_origin.squaredNorm() == 0.0)
            {
                return false;
            }
            gravity_state -= imu.mean_specific_force_at_state_origin.normalized();
        }
        if (gravity_state.squaredNorm() == 0.0 || params_.gravity_norm == 0.0) {
            return false;
        }
        gravity_state.normalize();
        gravity_state *= params_.gravity_norm;

        state_.timestamp = timestamp;
        state_.pose.setIdentity();
        state_.vel.setZero();
        state_.omg.setZero();
        if (params_.align_gravity_with_z_axis) {
            const Eigen::Vector3d gravity_odom = -params_.gravity_norm * Eigen::Vector3d::UnitZ();
            state_.pose.linear() = Eigen::Quaterniond::FromTwoVectors(
                                       gravity_state.normalized(),
                                       gravity_odom.normalized()
            )
                                       .normalized()
                                       .toRotationMatrix();
            state_.gravity = gravity_odom;
        } else {
            state_.gravity = gravity_state;
        }
        state_.acc = -state_.pose.linear().transpose() * state_.gravity;

        initial_points_.reserve(initial_points_.size() + pending_initial_points_.size());
        for (const auto& point: pending_initial_points_) {
            initial_points_.push_back((state_.pose * point.cast<double>()).cast<float>());
        }
        pending_initial_points_.clear();
        init_stage_ = InitStage::BuildingMap;
        return true;
    }

    Params params_;
    InitStage init_stage_ = InitStage::WaitingImu;
    std::vector<ImuState> imu_states_;
    EstimationState state_;
    Covariance P_ = Covariance::Zero();
    std::shared_ptr<SmallIVox> ivox_;
    MeasurementBatch batch_;
    std::vector<Eigen::Vector3f> initial_points_;
    std::vector<Eigen::Vector3f> pending_initial_points_;
    std::vector<Eigen::Vector3d> points_odom_cache_;
};

} // namespace sleepy
