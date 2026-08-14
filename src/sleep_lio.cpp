#include "sleep_lio.hpp"

#include "sensor/common.hpp"
#include "sensor/imu.hpp"
#include "sensor/point_cloud.hpp"
#include "odometry/continuous_time_odometry_estimation.hpp"
#include "utils/lock_queue.hpp"
#include "utils/logger.hpp"
#include "utils/rcl_tf.hpp"
#include "utils/rclcpp_parameter_node.hpp"
#include "utils/timed_reorder_buffer.hpp"
#include "utils/utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
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

struct ReorderSample {
  using key_type = std::tuple<std::uint8_t, std::size_t>;
  double timestamp = 0.0;
  std::uint8_t type_rank = 0;
  common::SensorTag sensor;
  struct SensorData {
    common::Point point;
    common::ImuMsg imu;
  };
  SensorData data;

  key_type order_key() const { return {type_rank, sensor.id}; }
};

} // namespace

struct SleepyLio::Impl {
  struct Params {
    double event_reorder_window = 0.03;
    std::string state_frame;
    void load(const utils::ParamsNode &config) {
      event_reorder_window =
          std::max(0.0, config.declare<double>("event_reorder_window",
                                               event_reorder_window));
      state_frame = config.declare<std::string>("state_frame", state_frame);
    }
  };

  explicit Impl(rclcpp::Node &node) : event_queue_(256) {
    tf_ = std::make_shared<utils::RclTF>(node);
    auto root_config = utils::ParamsNode(node);
    params_.load(root_config);
    odometry_ =
        std::make_unique<ContinuousTimeOdometryEstimation>(root_config);

    auto sensor_config = root_config.sub("sensor");

    const auto pointcloud_names =
        sensor_config.declare<std::vector<std::string>>("pointcloud_sensors");
    point_cloud_sensors_.reserve(pointcloud_names.size());
    for (std::size_t i = 0; i < pointcloud_names.size(); ++i) {
      const auto &name = pointcloud_names[i];
      auto pointcloud_config = sensor_config.sub(name);
      auto sensor = PointCloudSensor(pointcloud_config);
      sensor.name = name;
      if (!sensor.lidar_adapter) {
        utils::log_error("failed to create lidar adapter for sensor {}", name);
        continue;
      }

      const common::SensorTag sensor_tag{i};
      sensor.lidar_adapter->setup_subscription(
          &node, sensor.topic, sensor_tag,
          [this, sensor_tag](std::vector<common::Point> &pointcloud,
                             const rclcpp::Time &stamp) {
            if (pointcloud.empty()) {
              return;
            }
            auto &sensor = point_cloud_sensors_[sensor_tag.id];
            if (!sensor.frame_in_state) {
              auto T_opt = tf_->get_transform<double>(
                  params_.state_frame, sensor.frame_id, stamp,
                  rclcpp::Duration::from_seconds(0.1));
              if (T_opt) {
                sensor.frame_in_state = *T_opt;
              } else {
                return;
              }
            }
            TimedEvent event{
                .type = TimedEvent::Type::PointCloud,
                .sensor = sensor_tag,
                .pointcloud = std::move(pointcloud),
                .imu = {},
            };
            if (event_queue_.push(std::move(event))) {
              utils::log_warn("sleep_lio dropped oldest queued pointcloud "
                              "event from sensor {}",
                              sensor_tag.id);
            }
          });
      point_cloud_sensors_.emplace_back(std::move(sensor));
    }

    const auto imu_names =
        sensor_config.declare<std::vector<std::string>>("imu_sensors");
    imu_sensors_.reserve(imu_names.size());
    for (std::size_t i = 0; i < imu_names.size(); ++i) {
      const auto &name = imu_names[i];
      auto imu_config = sensor_config.sub(name);
      auto sensor = ImuSensor(imu_config);
      sensor.name = name;

      const common::SensorTag sensor_tag{i};
      sensor.imu_subscription = node.create_subscription<sensor_msgs::msg::Imu>(
          sensor.topic, rclcpp::SensorDataQoS(),
          [this, sensor_tag,
           acc_scale = sensor.acc_scale](const sensor_msgs::msg::Imu &msg) {
            auto &sensor = imu_sensors_[sensor_tag.id];
            if (!sensor.frame_in_state) {
              auto T_opt = tf_->get_transform<double>(
                  params_.state_frame, sensor.frame_id, msg.header.stamp,
                  rclcpp::Duration::from_seconds(0.1));
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
                  sensor_tag.id);
            }
          });
      imu_sensors_.emplace_back(std::move(sensor));
    }

    worker_ = std::thread([this] { worker_loop(); });
  }

  ~Impl() {
    event_queue_.stop();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void worker_loop() {
    utils::TimedReorderBuffer<ReorderSample> reorder_buffer(
        params_.event_reorder_window);

    auto handle_ready_sample = [this](ReorderSample &&sample) {
      handle_sample(std::move(sample));
    };

    while (true) {
      TimedEvent event;
      if (!event_queue_.wait_and_pop(event)) {
        break;
      }

      if (event.type == TimedEvent::Type::PointCloud) {

        for (auto &point : event.pointcloud) {
          ReorderSample sample{
              .timestamp = point.timestamp,
              .type_rank =
                  static_cast<std::uint8_t>(TimedEvent::Type::PointCloud),
              .sensor = event.sensor,
              .data =
                  {
                      .point = std::move(point),
                      .imu = {},
                  },
          };
          reorder_buffer.push(std::move(sample), handle_ready_sample);
        }
        continue;
      }

      ReorderSample sample{
          .timestamp = event.imu.timestamp,
          .type_rank = static_cast<std::uint8_t>(TimedEvent::Type::Imu),
          .sensor = event.sensor,
          .data =
              {
                  .point = {},
                  .imu = event.imu,
              },
      };
      reorder_buffer.push(std::move(sample), handle_ready_sample);
    }

    reorder_buffer.flush(handle_ready_sample);
  }

  void handle_sample(ReorderSample &&sample) {
    if (sample.timestamp < last_emitted_timestamp_) {
      utils::log_warn("out-of-order sample ...");
    }
    last_emitted_timestamp_ =
        std::max(last_emitted_timestamp_, sample.timestamp);
    switch (sample.type_rank) {
    case static_cast<std::uint8_t>(TimedEvent::Type::PointCloud): {
      ++sorted_point_count_;
      odometry_->add_point(sample.data.point, point_cloud_sensors_);
      break;
    }

    case static_cast<std::uint8_t>(TimedEvent::Type::Imu): {
      ++sorted_imu_count_;
      odometry_->add_imu(sample.data.imu, imu_sensors_);
      break;
    }
    }

    utils::dt_once(
        [&]() {
          utils::log_info("sorted_pt: {} imu: {}", sorted_point_count_,
                          sorted_imu_count_);
          sorted_point_count_ = 0;
          sorted_imu_count_ = 0;
        },
        std::chrono::duration<double>(1.0));
  }

  Params params_;
  utils::LockQueue<TimedEvent> event_queue_;
  std::thread worker_;
  std::vector<PointCloudSensor> point_cloud_sensors_;
  std::vector<ImuSensor> imu_sensors_;
  std::unique_ptr<ContinuousTimeOdometryEstimation> odometry_;
  std::size_t sorted_point_count_ = 0;
  std::size_t sorted_imu_count_ = 0;
  double last_emitted_timestamp_ = -1;
  utils::RclTF::Ptr tf_;
};

SleepyLio::SleepyLio(rclcpp::Node &node) {
  _impl = std::make_unique<Impl>(node);
}

SleepyLio::~SleepyLio() = default;

} // namespace sleepy
