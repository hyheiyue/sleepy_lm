#pragma once
#include <Eigen/Dense>
#include <rclcpp/node.hpp>
#include <string>
#include <utility>
#include <vector>
namespace sleepy::utils {
class ParamsNode {
public:
    ParamsNode(rclcpp::Node& node, std::string prefix = ""): node_(node) {
        if (!prefix.empty() && prefix.back() != '.')
            prefix += ".";
        prefix_ = std::move(prefix);
    }

    template<typename T>
    [[nodiscard]] T declare(const std::string& name, const T& default_value) const {
        const std::string full = prefix_ + name;

        if (node_.has_parameter(full)) {
            T value;
            node_.get_parameter(full, value);
            return value;
        }

        return node_.declare_parameter<T>(full, default_value);
    }
    template<typename T>
    [[nodiscard]] T declare(const std::string& name) const {
        const std::string full = prefix_ + name;

        if (node_.has_parameter(full)) {
            T value;
            node_.get_parameter(full, value);
            return value;
        }

        return node_.declare_parameter<T>(full);
    }

    [[nodiscard]] ParamsNode sub(const std::string& sub_prefix) const {
        return ParamsNode(node_, prefix_ + sub_prefix);
    }

private:
    rclcpp::Node& node_;
    std::string prefix_;
};

template<>
inline Eigen::Vector3d ParamsNode::declare<Eigen::Vector3d>(const std::string& name) const {
    const auto values = declare<std::vector<double>>(name);
    if (values.size() != 3) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d out(values[0], values[1], values[2]);
    return out.allFinite() ? out : Eigen::Vector3d::Zero();
}

template<>
inline Eigen::Matrix3d ParamsNode::declare<Eigen::Matrix3d>(const std::string& name) const {
    const auto values = declare<std::vector<double>>(name);
    if (values.size() != 9) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix3d out;
    out << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
        values[8];
    return out.allFinite() ? out : Eigen::Matrix3d::Identity();
}

} // namespace sleepy::utils
