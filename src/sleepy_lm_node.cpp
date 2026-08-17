#include "backward-cpp/backward.hpp"
#include "sleep_lm.hpp"
#include <memory>
namespace sleepy {
class SleepyLmNode: public rclcpp::Node {
public:
    SleepyLmNode(const rclcpp::NodeOptions& options): Node("sleepy_lm", options) {
        lio_ = std::make_unique<SleepyLm>(*this);
    }
    std::unique_ptr<SleepyLm> lio_;
};
} // namespace sleepy
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sleepy::SleepyLmNode)