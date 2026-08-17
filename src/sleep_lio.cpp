#include "sleep_lio.hpp"
#include "odometry/point_lio.hpp"
#include "sensor/common.hpp"
#include "sensor/imu.hpp"
#include "sensor/point_cloud.hpp"
#include "utils/lock_queue.hpp"
#include "utils/logger.hpp"
#include "utils/rcl_tf.hpp"
#include "utils/rclcpp_parameter_node.hpp"
#include "utils/utils.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tbb/tbb.h>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>
namespace sleepy {

namespace {

    struct TimedEvent {
        enum class Type : std::uint8_t { PointCloud = 0, Imu = 1 };
        Type type = Type::PointCloud;
        common::SensorTag sensor;
        std::vector<common::Point> pointcloud;
        common::ImuMsg imu;
    };

} // namespace

struct SleepyLio::Impl {
    struct Params {
        std::string odom_frame = "odom";
        std::string state_frame;
        int point_filter_num = 1;
        double min_distance_squared;
        double max_distance_squared;
        void load(const utils::ParamsNode& config) {
            odom_frame = config.declare<std::string>("odom_frame", odom_frame);
            state_frame = config.declare<std::string>("state_frame");
            min_distance_squared =
                config.declare<double>("min_distance") * config.declare<double>("min_distance");
            max_distance_squared =
                config.declare<double>("max_distance") * config.declare<double>("max_distance");
            point_filter_num = config.declare<int>("point_filter_num", point_filter_num);
        }
    };

    explicit Impl(rclcpp::Node& node) {
        tf_ = std::make_shared<utils::RclTF>(node);
        auto root_config = utils::ParamsNode(node);
        params_.load(root_config);
        point_lio_ = std::make_unique<PointLio>(root_config);
        odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        odom_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("odometry_path", 10);
        pointcloud_pub_ =
            node.create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        auto sensor_config = root_config.sub("sensor");

        const auto pointcloud_names =
            sensor_config.declare<std::vector<std::string>>("pointcloud_sensors");
        point_cloud_sensors_.reserve(pointcloud_names.size());
        for (std::size_t i = 0; i < pointcloud_names.size(); ++i) {
            const auto& name = pointcloud_names[i];
            auto pointcloud_config = sensor_config.sub(name);
            auto sensor = PointCloudSensor(pointcloud_config);
            sensor.name = name;
            if (!sensor.lidar_adapter) {
                utils::log_error("failed to create lidar adapter for sensor {}", name);
                continue;
            }

            const common::SensorTag sensor_tag { i };
            sensor.lidar_adapter->setup_subscription(
                &node,
                sensor.topic,
                sensor_tag,
                [this, sensor_tag](
                    std::vector<common::Point>& raw_pointcloud,
                    const rclcpp::Time& stamp
                ) {
                    if (raw_pointcloud.empty()) {
                        return;
                    }
                    log_.pc_cb++;
                    auto& sensor = point_cloud_sensors_[sensor_tag.id];
                    if (!sensor.frame_in_state) {
                        auto T_opt = tf_->get_transform<double>(
                            params_.state_frame,
                            sensor.frame_id,
                            stamp,
                            rclcpp::Duration::from_seconds(0.1)
                        );
                        if (T_opt) {
                            sensor.frame_in_state = *T_opt;
                        } else {
                            return;
                        }
                    }

                    TimedEvent event {
                        .type = TimedEvent::Type::PointCloud,
                        .sensor = sensor_tag,
                        .pointcloud = std::move(raw_pointcloud),
                        .imu = {},
                    };
                    if (event_queue_.push(std::move(event))) {
                        utils::log_warn(
                            "sleep_lio dropped oldest queued pointcloud "
                            "event from sensor {}",
                            sensor_tag.id
                        );
                    }
                }
            );
            point_cloud_sensors_.emplace_back(std::move(sensor));
        }
        preprocess_.point_deques.resize(pointcloud_names.size());
        preprocess_.last_timestamp_lidars.resize(pointcloud_names.size(), -1);
        const auto imu_names = sensor_config.declare<std::vector<std::string>>("imu_sensors");
        imu_sensors_.reserve(imu_names.size());
        for (std::size_t i = 0; i < imu_names.size(); ++i) {
            const auto& name = imu_names[i];
            auto imu_config = sensor_config.sub(name);
            auto sensor = ImuSensor(imu_config);
            sensor.name = name;

            const common::SensorTag sensor_tag { i };
            sensor.imu_subscription = node.create_subscription<sensor_msgs::msg::Imu>(
                sensor.topic,
                rclcpp::SensorDataQoS(),
                [this, sensor_tag, acc_scale = sensor.acc_scale](const sensor_msgs::msg::Imu& msg) {
                    log_.imu_cb++;
                    auto& sensor = imu_sensors_[sensor_tag.id];
                    if (!sensor.frame_in_state) {
                        auto T_opt = tf_->get_transform<double>(
                            params_.state_frame,
                            sensor.frame_id,
                            msg.header.stamp,
                            rclcpp::Duration::from_seconds(0.1)
                        );
                        if (T_opt) {
                            sensor.frame_in_state = *T_opt;
                        } else {
                            return;
                        }
                    }
                    TimedEvent event{
                .type = TimedEvent::Type::Imu,
                .sensor = sensor_tag,
                .pointcloud = {},
                .imu =
                    {
                        .timestamp = msg.header.stamp.sec +
                                     msg.header.stamp.nanosec / 1e9,
                        .linear_acceleration = Eigen::Vector3d(
                            msg.linear_acceleration.x * acc_scale,
                            msg.linear_acceleration.y * acc_scale,
                            msg.linear_acceleration.z * acc_scale),
                        .angular_velocity = Eigen::Vector3d(
                            msg.angular_velocity.x, msg.angular_velocity.y,
                            msg.angular_velocity.z),
                        .sensor = sensor_tag,
                    },
            };
                    if (event_queue_.push(std::move(event))) {
                        utils::log_warn(
                            "sleep_lio dropped oldest queued imu event from sensor {}",
                            sensor_tag.id
                        );
                    }
                }
            );
            imu_sensors_.emplace_back(std::move(sensor));
        }
        preprocess_.imu_deques.resize(imu_names.size());
        preprocess_.last_timestamp_imus.resize(imu_names.size(), -1);
        worker_ = std::thread([this] { worker_loop(); });
    }

    ~Impl() {
        event_queue_.stop();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void worker_loop() {
        while (true) {
            TimedEvent event;
            if (!event_queue_.wait_and_pop(event)) {
                break;
            }
            auto sensor_id = event.sensor.id;
            if (event.type == TimedEvent::Type::PointCloud) {
                auto& pointcloud = event.pointcloud;
                std::vector<common::Point> filtered_points;
                filtered_points.reserve(pointcloud.size());

                for (size_t i = 0; i < pointcloud.size(); i++) {
                    const auto& point = pointcloud[i];
                    if (point.timestamp < preprocess_.last_timestamp_lidars[sensor_id]) {
                        continue;
                    }
                    float dist = point.position.squaredNorm();
                    if (dist < params_.min_distance_squared || dist > params_.max_distance_squared)
                    {
                        continue;
                    }
                    if (i % params_.point_filter_num != 0) {
                        continue;
                    }
                    filtered_points.push_back(point);
                }
                auto timestamp_less = [](const auto& x, const auto& y) {
                    return x.timestamp < y.timestamp;
                };
                tbb::parallel_sort(filtered_points.begin(), filtered_points.end(), timestamp_less);
                if (!filtered_points.empty()) {
                    preprocess_.last_timestamp_lidars[sensor_id] = filtered_points.back().timestamp;
                    preprocess_.point_deques[sensor_id].insert(
                        preprocess_.point_deques[sensor_id].end(),
                        filtered_points.begin(),
                        filtered_points.end()
                    );
                }
            } else {
                auto imu = event.imu;
                if (imu.timestamp < preprocess_.last_timestamp_imus[sensor_id]) {
                    continue;
                }
                preprocess_.last_timestamp_imus[sensor_id] = imu.timestamp;
                preprocess_.imu_deques[sensor_id].push_back(imu);
            }
            handle_once();
        }
    }
    struct LogCtx {
        int pc_cb = 0;
        int imu_cb = 0;
        int processed_pts = 0;
        double cost_ms = 0;
        void reset() {
            *this = {};
        }
    } log_;
    void handle_once() {
        enum class SensorType { Lidar, Imu };

        struct EarlySensor {
            int id { -1 };
            SensorType type;
            double timestamp { std::numeric_limits<double>::max() };
        };

        auto all_sensor_ready = [&]() {
            double max_front = -std::numeric_limits<double>::infinity();
            double min_back = std::numeric_limits<double>::max();

            auto check_queue = [&](const auto& queues) {
                for (const auto& queue: queues) {
                    if (queue.empty()) {
                        return false;
                    }

                    max_front = std::max(max_front, queue.front().timestamp);
                    min_back = std::min(min_back, queue.back().timestamp);
                }
                return true;
            };

            return check_queue(preprocess_.point_deques) && check_queue(preprocess_.imu_deques)
                && max_front < min_back;
        };

        auto find_earliest_sensor = [&]() {
            EarlySensor result;

            auto check_queue = [&](const auto& queues, SensorType type) {
                for (int i = 0; i < queues.size(); ++i) {
                    const auto& front = queues[i].front();

                    if (front.timestamp < result.timestamp) {
                        result.timestamp = front.timestamp;
                        result.id = i;
                        result.type = type;
                    }
                }
            };

            check_queue(preprocess_.point_deques, SensorType::Lidar);

            check_queue(preprocess_.imu_deques, SensorType::Imu);

            return result;
        };

        bool processed_sample = false;
        while (all_sensor_ready()) {
            const auto sensor = find_earliest_sensor();

            if (sensor.id < 0) {
                break;
            }
            auto start = std::chrono::steady_clock::now();
            switch (sensor.type) {
                case SensorType::Lidar: {
                    auto& point = preprocess_.point_deques[sensor.id].front();
                    auto pt_odom_opt = point_lio_->add_point(point, point_cloud_sensors_);
                    if (pt_odom_opt) {
                        points_odom_.push_back(*pt_odom_opt);
                    }
                    preprocess_.point_deques[sensor.id].pop_front();
                    log_.processed_pts++;
                    processed_sample = true;
                    break;
                }

                case SensorType::Imu: {
                    auto& imu = preprocess_.imu_deques[sensor.id].front();
                    point_lio_->add_imu(imu, imu_sensors_);
                    preprocess_.imu_deques[sensor.id].pop_front();
                    processed_sample = true;
                    break;
                }
            }
            auto end = std::chrono::steady_clock::now();
            log_.cost_ms +=
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start)
                    .count();
        }

        if (processed_sample && point_lio_->state_initialized()) {
            publish_odometry();
            publish_pointcloud();
            points_odom_.clear();
        }

        utils::dt_once(
            [&]() {
                utils::log_info(
                    "pc: {} imu: {} pts: {} cost: {:.2f}ms",
                    log_.pc_cb,
                    log_.imu_cb,
                    log_.processed_pts,
                    log_.cost_ms
                );
                log_.reset();
            },
            std::chrono::duration<double>(1.0)
        );
    }

    void publish_odometry() {
        const auto& state = point_lio_->state();
        static double last_published_timestamp = -1;
        if (state.timestamp <= last_published_timestamp) {
            return;
        }
        last_published_timestamp = state.timestamp;

        const rclcpp::Time stamp(static_cast<std::int64_t>(state.timestamp * 1e9));
        const Eigen::Quaterniond orientation(state.pose.linear());
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = params_.odom_frame;
        odometry.child_frame_id = params_.state_frame;
        odometry.pose.pose.position.x = state.pose.translation().x();
        odometry.pose.pose.position.y = state.pose.translation().y();
        odometry.pose.pose.position.z = state.pose.translation().z();
        odometry.pose.pose.orientation.x = orientation.x();
        odometry.pose.pose.orientation.y = orientation.y();
        odometry.pose.pose.orientation.z = orientation.z();
        odometry.pose.pose.orientation.w = orientation.w();

        const Eigen::Vector3d velocity_state = state.pose.linear().transpose() * state.vel;
        odometry.twist.twist.linear.x = velocity_state.x();
        odometry.twist.twist.linear.y = velocity_state.y();
        odometry.twist.twist.linear.z = velocity_state.z();
        odometry.twist.twist.angular.x = state.omg.x();
        odometry.twist.twist.angular.y = state.omg.y();
        odometry.twist.twist.angular.z = state.omg.z();
        odom_pub_->publish(odometry);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odometry.header;
        pose.pose = odometry.pose.pose;
        odom_path_.header = odometry.header;
        odom_path_.poses.push_back(std::move(pose));
        constexpr std::size_t max_path_size = 10000;
        if (odom_path_.poses.size() > max_path_size) {
            odom_path_.poses.erase(odom_path_.poses.begin());
        }
        odom_path_pub_->publish(odom_path_);
        tf_->publish_transform(state.pose, params_.odom_frame, params_.state_frame, stamp);
    }
    void publish_pointcloud() {
        if (points_odom_.empty()) {
            return;
        }
        const auto& state = point_lio_->state();
        static double last_published_timestamp = -1;
        if (state.timestamp <= last_published_timestamp) {
            return;
        }
        last_published_timestamp = state.timestamp;

        sensor_msgs::msg::PointCloud2 msg;

        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(state.timestamp * 1e9));

        msg.header.frame_id = params_.odom_frame;

        sensor_msgs::PointCloud2Modifier modifier(msg);

        modifier.setPointCloud2Fields(
            4,
            "x",
            1,
            sensor_msgs::msg::PointField::FLOAT32,

            "y",
            1,
            sensor_msgs::msg::PointField::FLOAT32,

            "z",
            1,
            sensor_msgs::msg::PointField::FLOAT32,

            "intensity",
            1,
            sensor_msgs::msg::PointField::FLOAT32
        );

        modifier.resize(points_odom_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
        for (const auto& point: points_odom_) {
            *iter_x = static_cast<float>(point.x());
            *iter_y = static_cast<float>(point.y());
            *iter_z = static_cast<float>(point.z());
            *iter_i = (point-state.pose.translation()).norm();
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_i;
        }

        pointcloud_pub_->publish(msg);
    }
    struct Preprocess {
        std::vector<std::deque<common::Point>> point_deques;
        std::vector<double> last_timestamp_lidars;
        std::vector<std::deque<common::ImuMsg>> imu_deques;
        std::vector<double> last_timestamp_imus;
    } preprocess_;
    Params params_;
    utils::LockQueue<TimedEvent> event_queue_;
    std::thread worker_;
    std::vector<PointCloudSensor> point_cloud_sensors_;
    std::vector<ImuSensor> imu_sensors_;
    std::unique_ptr<PointLio> point_lio_;
    utils::RclTF::Ptr tf_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    nav_msgs::msg::Path odom_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    std::vector<Eigen::Vector3d> points_odom_;
};

SleepyLio::SleepyLio(rclcpp::Node& node) {
    _impl = std::make_unique<Impl>(node);
}

SleepyLio::~SleepyLio() = default;

} // namespace sleepy
