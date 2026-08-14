#pragma once
#include "base_lidar.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace sleepy {
// RoboSense 点云适配器假设输入包含 x/y/z 和 double timestamp 字段。
class RoboSenseLidar: public LidarAdapterBase {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

public:
    inline void setup_subscription(
        rclcpp::Node* node,
        const std::string& topic,
        const common::SensorTag& sensor,
        std::function<void(std::vector<common::Point>&, const rclcpp::Time&)> callback
    ) override {
        subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic,
            rclcpp::SensorDataQoS(),
            [callback, sensor](const sensor_msgs::msg::PointCloud2& msg) {
                // RoboSense timestamp 是点级绝对时间，单位为秒，直接用于 IMU
                // 状态插值。
                sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
                sensor_msgs::PointCloud2ConstIterator<double> out_timestamp(msg, "timestamp");
                size_t size = msg.width * msg.height;
                std::vector<common::Point> pointcloud;
                pointcloud.reserve(size);
                for (size_t i = 0; i < size; ++i) {
                    pointcloud.emplace_back();
                    auto& new_point = pointcloud.back();
                    new_point.position << *out_x, *out_y, *out_z;
                    new_point.timestamp = *out_timestamp;
                    new_point.sensor = sensor;
                    ++out_x;
                    ++out_y;
                    ++out_z;
                    ++out_timestamp;
                }
                callback(pointcloud, msg.header.stamp);
            }
        );
    }
};
} // namespace sleepy
