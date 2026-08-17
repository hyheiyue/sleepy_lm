#pragma once
#include <memory>
#include <rclcpp/node.hpp>
namespace sleepy {
class SleepyLm {
public:
    SleepyLm(rclcpp::Node& node);
    ~SleepyLm();
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
} // namespace sleepy
