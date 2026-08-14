#include "backward-cpp/backward.hpp"
#include "sleep_lio.hpp"
#include <memory>
namespace sleepy {
class SleepyLioNode: public rclcpp::Node {
public:
    SleepyLioNode(const rclcpp::NodeOptions& options): Node("sleepy_lio", options) {
        lio_ = std::make_unique<SleepyLio>(*this);
    }
    std::unique_ptr<SleepyLio> lio_;
};
} // namespace sleepy
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sleepy::SleepyLioNode)