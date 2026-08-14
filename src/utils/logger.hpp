#pragma once

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace sleepy::utils {

inline const rclcpp::Logger& logger() {
    static const rclcpp::Logger logger = rclcpp::get_logger("sleepy_lio");
    return logger;
}

template<typename... Args>
inline std::string format_message(fmt::format_string<Args...> format, Args&&... args) {
    return fmt::format(format, std::forward<Args>(args)...);
}

template<typename... Args>
inline void log_debug(fmt::format_string<Args...> format, Args&&... args) {
    const std::string message = format_message(format, std::forward<Args>(args)...);
    RCLCPP_DEBUG(logger(), "%s", message.c_str());
}

template<typename... Args>
inline void log_info(fmt::format_string<Args...> format, Args&&... args) {
    const std::string message = format_message(format, std::forward<Args>(args)...);
    RCLCPP_INFO(logger(), "%s", message.c_str());
}

template<typename... Args>
inline void log_warn(fmt::format_string<Args...> format, Args&&... args) {
    const std::string message = format_message(format, std::forward<Args>(args)...);
    RCLCPP_WARN(logger(), "%s", message.c_str());
}

template<typename... Args>
inline void log_error(fmt::format_string<Args...> format, Args&&... args) {
    const std::string message = format_message(format, std::forward<Args>(args)...);
    RCLCPP_ERROR(logger(), "%s", message.c_str());
}

template<typename... Args>
inline void log_fatal(fmt::format_string<Args...> format, Args&&... args) {
    const std::string message = format_message(format, std::forward<Args>(args)...);
    RCLCPP_FATAL(logger(), "%s", message.c_str());
}

} // namespace sleepy::utils
