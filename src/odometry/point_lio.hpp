#pragma once

#include "ivox.hpp"
#include "sensor/imu.hpp"
#include "sensor/point_cloud.hpp"
#include "state.hpp"
#include "utils/logger.hpp"
#include "utils/rclcpp_parameter_node.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace sleepy {

class PointLio {
private:
    static constexpr int POSITION_INDEX = 0;
    static constexpr int ROTATION_INDEX = 3;
    static constexpr int VELOCITY_INDEX = 6;
    static constexpr int OMEGA_INDEX = 9;
    static constexpr int ACCELERATION_INDEX = 12;
    static constexpr int GRAVITY_INDEX = 15;
    static constexpr int BASE_STATE_DIM = 18;
    static constexpr int IMU_ERROR_DIM = 6;
    static constexpr std::size_t MAX_IMUS = 4;
    static constexpr int MAX_ERROR_STATE_DIM =
        BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * MAX_IMUS);
    static constexpr std::size_t NUM_MATCH_POINTS = 5;

    using Covariance = Eigen::Matrix<double, MAX_ERROR_STATE_DIM, MAX_ERROR_STATE_DIM>;
    using ErrorState = Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 1>;

    struct Params {
        double gravity_norm = 9.80665;
        bool align_gravity_with_z_axis = true;
        int initialization_min_samples = 200;
        double initialization_window = 1.0;
        int initial_map_size = 1000;
        double map_resolution = 0.1;
        int map_capacity = 1000000;
        double laser_point_cov = 0.01;
        double imu_meas_acc_cov = 0.01;
        double imu_meas_omg_cov = 0.01;
        double velocity_cov = 20.0;
        double acceleration_cov = 500.0;
        double omg_cov = 1000.0;
        double ba_cov = 0.0001;
        double bg_cov = 0.0001;
        double plane_threshold = 0.1;
        double match_sqaured = 81.0;
        bool check_satu = false;
        double satu_acc = std::numeric_limits<double>::infinity();
        double satu_gyro = std::numeric_limits<double>::infinity();

        void load(const utils::ParamsNode& config) {
            gravity_norm = config.declare<double>("gravity_norm");
            align_gravity_with_z_axis = config.declare<bool>("align_gravity_with_z_axis");
            initialization_min_samples = config.declare<int>("initialization_min_samples");
            initialization_window = config.declare<double>("initialization_window");
            initial_map_size = config.declare<int>("initial_map_size");
            map_resolution = config.declare<double>("map_resolution");
            map_capacity = config.declare<int>("map_capacity");
            laser_point_cov = config.declare<double>("laser_point_cov");
            imu_meas_acc_cov = config.declare<double>("imu_meas_acc_cov");
            imu_meas_omg_cov = config.declare<double>("imu_meas_omg_cov");
            velocity_cov = config.declare<double>("velocity_cov");
            acceleration_cov = config.declare<double>("acceleration_cov");
            omg_cov = config.declare<double>("omg_cov");
            ba_cov = config.declare<double>("ba_cov");
            bg_cov = config.declare<double>("bg_cov");
            plane_threshold = config.declare<double>("plane_threshold");
            match_sqaured = config.declare<double>("match_sqaured");
            check_satu = config.declare<bool>("check_satu");
            satu_acc = config.declare<double>("satu_acc");
            satu_gyro = config.declare<double>("satu_gyro");
        }
    };

    enum class InitStage { WaitingImu, BuildingMap, Finished };

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

public:
    explicit PointLio(const utils::ParamsNode& config) {
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

    [[nodiscard]] bool map_initialized() const noexcept {
        return init_stage_ == InitStage::Finished;
    }

    void add_imu(const common::ImuMsg& imu, const std::vector<ImuSensor>& sensors) {
        if (imu.sensor.id >= sensors.size() || sensors.size() > MAX_IMUS
            || !std::isfinite(imu.timestamp) || !imu.linear_acceleration.allFinite()
            || !imu.angular_velocity.allFinite())
        {
            return;
        }

        ensure_imu_count(sensors.size());
        const auto& sensor = sensors[imu.sensor.id];
        if (!sensor.frame_in_state) {
            return;
        }

        auto& imu_state = imu_states_[imu.sensor.id];
        imu_state.imu_in_state = sensor.frame_in_state.value() * sensor.sensor_in_frame;

        if (!imu_state.initialized) {
            collect_initialization_sample(imu_state, imu);
            if (all_imu_initialized()) {
                initialize_state(imu.timestamp);
            }
            return;
        }

        if (!state_initialized() || imu.timestamp < state_.timestamp) {
            return;
        }

        predict_to(imu.timestamp);
        update_imu(imu, imu_state, imu.sensor.id);
    }

    std::optional<Eigen::Vector3d>
    add_point(const common::Point& point_in_lidar, const std::vector<PointCloudSensor>& sensors) {
        if (point_in_lidar.sensor.id >= sensors.size() || !std::isfinite(point_in_lidar.timestamp)
            || !point_in_lidar.position.allFinite())
        {
            return std::nullopt;
        }

        const auto& sensor = sensors[point_in_lidar.sensor.id];
        if (!sensor.frame_in_state) {
            return std::nullopt;
        }

        const Eigen::Isometry3d lidar_in_state =
            sensor.frame_in_state.value() * sensor.sensor_in_frame;
        const Eigen::Vector3d point_state = lidar_in_state * point_in_lidar.position.cast<double>();

        if (!state_initialized()) {
            // Keep the startup scan in the fixed state frame. It is transformed
            // into odometry only after gravity has established the initial pose.
            pending_initial_points_.push_back(point_state.cast<float>());
            return std::nullopt;
        }

        if (point_in_lidar.timestamp < state_.timestamp) {
            return std::nullopt;
        }
        predict_to(point_in_lidar.timestamp);

        if (init_stage_ == InitStage::BuildingMap) {
            initial_points_.push_back(transform_point_to_odom(point_state).cast<float>());
            initialize_map();
            return std::nullopt;
        }

        if (init_stage_ != InitStage::Finished) {
            return std::nullopt;
        }

        update_point(point_in_lidar.position, point_state);
        auto point_odom = transform_point_to_odom(point_state);
        ivox_->add_point(point_odom.cast<float>());
        return std::make_optional(point_odom);
    }

private:
    static Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
        Eigen::Matrix3d out;
        out << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return out;
    }

    static Eigen::Matrix3d exp_so3(const Eigen::Vector3d& angle) {
        const double norm = angle.norm();
        if (norm < 1e-12) {
            return Eigen::Matrix3d::Identity() + hat(angle);
        }
        return Eigen::AngleAxisd(norm, angle / norm).toRotationMatrix();
    }

    static Eigen::Matrix3d a_matrix(const Eigen::Vector3d& angle) {
        const double squared_norm = angle.squaredNorm();
        if (squared_norm < 1e-22) {
            return Eigen::Matrix3d::Identity();
        }
        const double norm = std::sqrt(squared_norm);
        const Eigen::Matrix3d angle_hat = hat(angle);
        return Eigen::Matrix3d::Identity() + (1.0 - std::cos(norm)) / squared_norm * angle_hat
            + (1.0 - std::sin(norm) / norm) / squared_norm * angle_hat * angle_hat;
    }

    static int bg_index(std::size_t imu_id) {
        return BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * imu_id);
    }

    static int ba_index(std::size_t imu_id) {
        return bg_index(imu_id) + 3;
    }

    [[nodiscard]] int active_error_state_dim() const {
        return BASE_STATE_DIM + static_cast<int>(IMU_ERROR_DIM * imu_states_.size());
    }

    void ensure_imu_count(std::size_t count) {
        if (count > MAX_IMUS || imu_states_.size() >= count) {
            return;
        }

        const std::size_t old_count = imu_states_.size();
        imu_states_.resize(count);

        if (old_count == 0) {
            P_.setZero();
            P_.topLeftCorner(BASE_STATE_DIM, BASE_STATE_DIM).diagonal().setConstant(0.01);
            P_.block<3, 3>(GRAVITY_INDEX, GRAVITY_INDEX).diagonal().setConstant(0.0001);
        }
        for (std::size_t i = old_count; i < imu_states_.size(); ++i) {
            P_.block<3, 3>(bg_index(i), bg_index(i)).diagonal().setConstant(0.001);
            P_.block<3, 3>(ba_index(i), ba_index(i)).diagonal().setConstant(0.001);
        }
        rebuild_process_noise();
    }

    void rebuild_process_noise() {
        process_noise_diagonal_.setZero();
        process_noise_diagonal_.segment<3>(VELOCITY_INDEX).setConstant(params_.velocity_cov);
        process_noise_diagonal_.segment<3>(OMEGA_INDEX).setConstant(params_.omg_cov);
        process_noise_diagonal_.segment<3>(ACCELERATION_INDEX)
            .setConstant(params_.acceleration_cov);
        for (std::size_t i = 0; i < imu_states_.size(); ++i) {
            process_noise_diagonal_.segment<3>(bg_index(i)).setConstant(params_.bg_cov);
            process_noise_diagonal_.segment<3>(ba_index(i)).setConstant(params_.ba_cov);
        }
    }

    void collect_initialization_sample(ImuState& imu_state, const common::ImuMsg& imu) {
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

        for (const auto& point: pending_initial_points_) {
            initial_points_.push_back(transform_point_to_odom(point.cast<double>()).cast<float>());
        }
        pending_initial_points_.clear();

        for (auto& imu: imu_states_) {
            imu.has_last_motion = false;
            imu.last_motion_timestamp = timestamp;
        }
        init_stage_ = InitStage::BuildingMap;

        return true;
    }

    bool initialize_map() {
        if (initial_points_.size() < static_cast<std::size_t>(params_.initial_map_size)) {
            return false;
        }
        for (const auto& point: initial_points_) {
            ivox_->add_point(point);
        }
        initial_points_.clear();
        init_stage_ = InitStage::Finished;
        utils::log_info("Point-LIO map initialized");
        return true;
    }

    [[nodiscard]] bool all_imu_initialized() const {
        return !imu_states_.empty()
            && std::all_of(imu_states_.begin(), imu_states_.end(), [](const ImuState& imu) {
                   return imu.initialized;
               });
    }

    void predict_to(double timestamp) {
        const double dt = timestamp - state_.timestamp;
        if (dt <= 0.0) {
            return;
        }

        const int active_dim = active_error_state_dim();
        const Eigen::Matrix3d R_odom_state = state_.pose.linear();
        const Eigen::Vector3d rotation_increment = state_.omg * dt;
        const Eigen::Matrix3d rotation_transition = exp_so3(-rotation_increment);
        const Eigen::Matrix3d omega_transition = a_matrix(-rotation_increment) * dt;
        const Eigen::Matrix3d velocity_rotation_transition = -R_odom_state * hat(state_.acc) * dt;
        const Eigen::Matrix3d velocity_acceleration_transition = R_odom_state * dt;

        // F differs from identity only in the position, rotation and velocity
        // block rows. Form FP and then the corresponding columns of FPF^T.
        Covariance FP;
        FP.topLeftCorner(active_dim, active_dim) = P_.topLeftCorner(active_dim, active_dim);
        FP.block(POSITION_INDEX, 0, 3, active_dim) = P_.block(POSITION_INDEX, 0, 3, active_dim)
            + dt * P_.block(VELOCITY_INDEX, 0, 3, active_dim);
        FP.block(ROTATION_INDEX, 0, 3, active_dim).noalias() =
            rotation_transition * P_.block(ROTATION_INDEX, 0, 3, active_dim)
            + omega_transition * P_.block(OMEGA_INDEX, 0, 3, active_dim);
        FP.block(VELOCITY_INDEX, 0, 3, active_dim).noalias() =
            P_.block(VELOCITY_INDEX, 0, 3, active_dim)
            + velocity_rotation_transition * P_.block(ROTATION_INDEX, 0, 3, active_dim)
            + velocity_acceleration_transition * P_.block(ACCELERATION_INDEX, 0, 3, active_dim)
            + dt * P_.block(GRAVITY_INDEX, 0, 3, active_dim);

        Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 3> position_columns;
        Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 3> rotation_columns;
        Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 3> velocity_columns;
        position_columns.topRows(active_dim) = FP.block(0, POSITION_INDEX, active_dim, 3)
            + dt * FP.block(0, VELOCITY_INDEX, active_dim, 3);
        rotation_columns.topRows(active_dim).noalias() =
            FP.block(0, ROTATION_INDEX, active_dim, 3) * rotation_transition.transpose()
            + FP.block(0, OMEGA_INDEX, active_dim, 3) * omega_transition.transpose();
        velocity_columns.topRows(active_dim).noalias() = FP.block(0, VELOCITY_INDEX, active_dim, 3)
            + FP.block(0, ROTATION_INDEX, active_dim, 3) * velocity_rotation_transition.transpose()
            + FP.block(0, ACCELERATION_INDEX, active_dim, 3)
                * velocity_acceleration_transition.transpose()
            + dt * FP.block(0, GRAVITY_INDEX, active_dim, 3);

        P_.topLeftCorner(active_dim, active_dim) = FP.topLeftCorner(active_dim, active_dim);
        P_.block(0, POSITION_INDEX, active_dim, 3) = position_columns.topRows(active_dim);
        P_.block(0, ROTATION_INDEX, active_dim, 3) = rotation_columns.topRows(active_dim);
        P_.block(0, VELOCITY_INDEX, active_dim, 3) = velocity_columns.topRows(active_dim);

        const double dt_squared = dt * dt;
        for (int i = 0; i < active_dim; ++i) {
            P_(i, i) += process_noise_diagonal_[i] * dt_squared;
        }
        symmetrize_active_covariance();

        state_.pose.translation() += state_.vel * dt;
        state_.vel += (R_odom_state * state_.acc + state_.gravity) * dt;
        state_.pose.linear() = R_odom_state * exp_so3(rotation_increment);
        state_.timestamp = timestamp;
    }

    void update_imu(const common::ImuMsg& imu, ImuState& imu_state, std::size_t imu_id) {
        const Eigen::Matrix3d R_state_imu = imu_state.imu_in_state.linear();
        const Eigen::Matrix3d R_imu_state = R_state_imu.transpose();
        const Eigen::Vector3d r_state_imu = imu_state.imu_in_state.translation();
        const Eigen::Vector3d measured_omega_state =
            R_state_imu * (imu.angular_velocity - imu_state.bg);

        Eigen::Vector3d alpha_state = Eigen::Vector3d::Zero();
        const double motion_dt = imu.timestamp - imu_state.last_motion_timestamp;
        if (imu_state.has_last_motion && motion_dt > 0.0) {
            alpha_state = (measured_omega_state - imu_state.last_omega_state) / motion_dt;
        }

        const Eigen::Vector3d lever =
            alpha_state.cross(r_state_imu) + state_.omg.cross(state_.omg.cross(r_state_imu));
        Eigen::Matrix<double, 6, 1> residual;
        residual.head<3>() = imu.angular_velocity - (R_imu_state * state_.omg + imu_state.bg);
        residual.tail<3>() =
            imu.linear_acceleration - (R_imu_state * (state_.acc + lever) + imu_state.ba);

        // Active columns are [omega, acceleration, bg_i, ba_i]. Keeping this
        // compact avoids multiplying the six-row model by all inactive IMU blocks.
        Eigen::Matrix<double, 6, 12> H = Eigen::Matrix<double, 6, 12>::Zero();
        H.block<3, 3>(0, 0) = R_imu_state;
        H.block<3, 3>(0, 6).setIdentity();
        const Eigen::Matrix3d centripetal_jacobian =
            state_.omg.dot(r_state_imu) * Eigen::Matrix3d::Identity()
            + state_.omg * r_state_imu.transpose() - 2.0 * r_state_imu * state_.omg.transpose();
        H.block<3, 3>(3, 0) = R_imu_state * centripetal_jacobian;
        H.block<3, 3>(3, 3) = R_imu_state;
        H.block<3, 3>(3, 9).setIdentity();

        Eigen::Matrix<double, 6, 6> measurement_cov = Eigen::Matrix<double, 6, 6>::Zero();
        measurement_cov.diagonal().head<3>().setConstant(params_.imu_meas_omg_cov);
        measurement_cov.diagonal().tail<3>().setConstant(params_.imu_meas_acc_cov);
        if (params_.check_satu) {
            for (int i = 0; i < 3; ++i) {
                if (std::abs(imu.angular_velocity[i]) >= params_.satu_gyro) {
                    H.row(i).setZero();
                    residual[i] = 0.0;
                }
                if (std::abs(imu.linear_acceleration[i]) >= params_.satu_acc) {
                    H.row(i + 3).setZero();
                    residual[i + 3] = 0.0;
                }
            }
        }

        const int active_dim = active_error_state_dim();
        Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 12> selected_columns;
        selected_columns.block(0, 0, active_dim, 3) = P_.block(0, OMEGA_INDEX, active_dim, 3);
        selected_columns.block(0, 3, active_dim, 3) =
            P_.block(0, ACCELERATION_INDEX, active_dim, 3);
        selected_columns.block(0, 6, active_dim, 3) = P_.block(0, bg_index(imu_id), active_dim, 3);
        selected_columns.block(0, 9, active_dim, 3) = P_.block(0, ba_index(imu_id), active_dim, 3);

        Eigen::Matrix<double, 12, 12> selected_covariance;
        selected_covariance.block<3, 12>(0, 0) = selected_columns.block<3, 12>(OMEGA_INDEX, 0);
        selected_covariance.block<3, 12>(3, 0) =
            selected_columns.block<3, 12>(ACCELERATION_INDEX, 0);
        selected_covariance.block<3, 12>(6, 0) = selected_columns.block(bg_index(imu_id), 0, 3, 12);
        selected_covariance.block<3, 12>(9, 0) = selected_columns.block(ba_index(imu_id), 0, 3, 12);

        Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 6> PHT;
        PHT.topRows(active_dim).noalias() = selected_columns.topRows(active_dim) * H.transpose();
        const Eigen::Matrix<double, 6, 6> innovation_covariance =
            H * selected_covariance * H.transpose() + measurement_cov;
        const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(innovation_covariance);
        if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
            Eigen::Matrix<double, MAX_ERROR_STATE_DIM, 6> gain;
            gain.topRows(active_dim).noalias() =
                PHT.topRows(active_dim) * ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());

            ErrorState correction = ErrorState::Zero();
            correction.head(active_dim).noalias() = gain.topRows(active_dim) * residual;
            apply_error_state(correction);

            P_.topLeftCorner(active_dim, active_dim).noalias() -= gain.topRows(active_dim)
                * innovation_covariance * gain.topRows(active_dim).transpose();
            symmetrize_active_covariance();
        }
        imu_state.last_omega_state = R_state_imu * (imu.angular_velocity - imu_state.bg);
        imu_state.last_motion_timestamp = imu.timestamp;
        imu_state.has_last_motion = true;
    }

    bool update_point(const Eigen::Vector3f& point_lidar, const Eigen::Vector3d& point_state) {
        const Eigen::Vector3f point_odom = transform_point_to_odom(point_state).cast<float>();
        nearest_points_.clear();
        ivox_->get_closest_point(point_odom, nearest_points_, NUM_MATCH_POINTS);
        if (nearest_points_.size() != NUM_MATCH_POINTS) {
            return false;
        }

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& point: nearest_points_) {
            centroid += point;
        }
        centroid /= static_cast<float>(nearest_points_.size());

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto& point: nearest_points_) {
            const Eigen::Vector3f centered = point - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
        covariance /= static_cast<float>(nearest_points_.size() - 1);
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        if (solver.info() != Eigen::Success) {
            return false;
        }

        const Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
        const float d = -normal.dot(centroid);
        for (const auto& point: nearest_points_) {
            if (std::abs(normal.dot(point) + d) > params_.plane_threshold) {
                return false;
            }
        }

        const double point_distance = static_cast<double>(normal.dot(point_odom) + d);
        if (point_lidar.norm() <= params_.match_sqaured * point_distance * point_distance) {
            return false;
        }

        const Eigen::Vector3d normal_odom = normal.cast<double>();
        const Eigen::Vector3d normal_state = state_.pose.linear().transpose() * normal_odom;
        Eigen::Matrix<double, 1, 6> H;
        H << normal_odom.transpose(), point_state.cross(normal_state).transpose();

        const int active_dim = active_error_state_dim();
        ErrorState PHT;
        PHT.head(active_dim).noalias() = P_.block(0, 0, active_dim, 6) * H.transpose();
        const double innovation_covariance =
            (H * P_.topLeftCorner<6, 6>() * H.transpose())(0, 0) + params_.laser_point_cov;
        if (!std::isfinite(innovation_covariance) || innovation_covariance <= 0.0) {
            return false;
        }

        ErrorState correction = ErrorState::Zero();
        correction.head(active_dim) =
            PHT.head(active_dim) * (-point_distance / innovation_covariance);
        apply_error_state(correction);

        P_.topLeftCorner(active_dim, active_dim).noalias() -=
            PHT.head(active_dim) * PHT.head(active_dim).transpose() / innovation_covariance;
        symmetrize_active_covariance();
        return true;
    }

    void symmetrize_active_covariance() {
        const int active_dim = active_error_state_dim();
        for (int row = 0; row < active_dim; ++row) {
            for (int col = 0; col < row; ++col) {
                const double average = 0.5 * (P_(row, col) + P_(col, row));
                P_(row, col) = average;
                P_(col, row) = average;
            }
        }
    }

    void apply_error_state(const ErrorState& correction) {
        state_.pose.translation() += correction.segment<3>(POSITION_INDEX);
        state_.pose.linear() *= exp_so3(correction.segment<3>(ROTATION_INDEX));
        state_.pose.linear() =
            Eigen::Quaterniond(state_.pose.linear()).normalized().toRotationMatrix();
        state_.vel += correction.segment<3>(VELOCITY_INDEX);
        state_.omg += correction.segment<3>(OMEGA_INDEX);
        state_.acc += correction.segment<3>(ACCELERATION_INDEX);
        state_.gravity += correction.segment<3>(GRAVITY_INDEX);
        for (std::size_t i = 0; i < imu_states_.size(); ++i) {
            imu_states_[i].bg += correction.segment<3>(bg_index(i));
            imu_states_[i].ba += correction.segment<3>(ba_index(i));
        }
    }

    [[nodiscard]] Eigen::Vector3d transform_point_to_odom(const Eigen::Vector3d& point_state
    ) const {
        return state_.pose * point_state;
    }

private:
    Params params_;
    InitStage init_stage_ = InitStage::WaitingImu;
    std::vector<ImuState> imu_states_;
    EstimationState state_;
    Covariance P_ = Covariance::Zero();
    ErrorState process_noise_diagonal_ = ErrorState::Zero();
    std::shared_ptr<SmallIVox> ivox_;
    std::vector<Eigen::Vector3f> initial_points_;
    std::vector<Eigen::Vector3f> pending_initial_points_;
    std::vector<Eigen::Vector3f> nearest_points_;
};

} // namespace sleepy
