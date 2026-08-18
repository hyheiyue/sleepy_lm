#pragma once
#include "lidar_adapter/base_lidar.hpp"
#include "lidar_adapter/livox_custom_msg.hpp"
#include "lidar_adapter/rslidar.hpp"
#include "sensor/common.hpp"
#include "utils/logger.hpp"
#include "utils/rclcpp_parameter_node.hpp"
#include "utils/utils.hpp"
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace sleepy {

struct PointCloudSensor {
    using Ptr = std::shared_ptr<PointCloudSensor>;
    std::string name;
    std::string frame_id;
    std::string type;
    std::string topic;
    Eigen::Isometry3d sensor_in_frame = Eigen::Isometry3d::Identity();
    std::optional<Eigen::Isometry3d> frame_in_state;
    std::unique_ptr<LidarAdapterBase> lidar_adapter;
    std::unique_ptr<LidarAdapterBase> make_lidar_adapter(const std::string& type) {
        const std::string normalized_type = utils::to_upper(type);
        if (normalized_type == "RSLIDAR") {
            return std::make_unique<RoboSenseLidar>();
        }
        if (normalized_type == "LIVOX_CUSTOM_MSG") {
#ifdef HAVE_LIVOX_DRIVER
            return std::make_unique<LivoxCustomMsgAdapter>();
#else
            utils::log_error("livox_custom_msg requested but not available!");
            return nullptr;
#endif
        }
        utils::log_error("unknown lidar type: {}", type);
        return nullptr;
    }
    PointCloudSensor(const utils::ParamsNode& config) {
        topic = config.declare<std::string>("topic");
        frame_id = config.declare<std::string>("frame_id");
        sensor_in_frame.translation() = config.declare<Eigen::Vector3d>("sensor_in_frame_t");
        sensor_in_frame.linear() = config.declare<Eigen::Matrix3d>("sensor_in_frame_R");
        type = config.declare<std::string>("type");
        lidar_adapter = make_lidar_adapter(type);
    }
};
} // namespace sleepy
