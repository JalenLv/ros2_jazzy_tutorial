#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"

class SimpleNode : public rclcpp::Node {
public:
    SimpleNode(const std::string &node_name) : Node(node_name) {}
};

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit LifecycleTalker(
        const std::string &node_name, bool intra_process_comms = false
    ) : rclcpp_lifecycle::LifecycleNode(
        node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    ) {}
};

using MyNodeInterfaces =rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeLoggingInterface
>;

void node_info(MyNodeInterfaces interfaces) {
    auto base_interface = interfaces.get_node_base_interface();
    auto logging_interface = interfaces.get_node_logging_interface();
    RCLCPP_INFO_STREAM(logging_interface->get_logger(), "Node name: " << base_interface->get_name());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<SimpleNode>("simple_node");
    auto lc_node = std::make_shared<LifecycleTalker>("simple_lifecycle_node");
    node_info(*node);
    node_info(*lc_node);

    rclcpp::shutdown();
    return 0;
}
