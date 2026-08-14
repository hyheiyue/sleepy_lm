#pragma once

#include "sensor/common.hpp"
#include <functional>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

namespace sleepy {

class LidarAdapterBase {
public:
    virtual ~LidarAdapterBase() = default;
    // 各 LiDAR 只负责把厂商点云字段转换成 common::Point 序列。
    virtual void setup_subscription(
        rclcpp::Node* node,
        const std::string& topic,
        const common::SensorTag& sensor,
        std::function<void(std::vector<common::Point>&, const rclcpp::Time&)> callback
    ) = 0;
};

} // namespace sleepy
