#pragma once
#include <memory>
#include <rclcpp/node.hpp>
namespace sleepy {
class SleepyLio {
public:
    SleepyLio(rclcpp::Node& node);
    ~SleepyLio();
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
} // namespace sleepy
