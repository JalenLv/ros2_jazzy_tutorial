#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

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

void node_info(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface
) {
    RCLCPP_INFO_STREAM(logging_interface->get_logger(), "Node name: " << base_interface->get_name());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<SimpleNode>("simple_node");
    auto lc_node = std::make_shared<LifecycleTalker>("simple_lifecycle_node");
    node_info(node->get_node_base_interface(), node->get_node_logging_interface());
    node_info(lc_node->get_node_base_interface(), lc_node->get_node_logging_interface());

    rclcpp::shutdown();
    return 0;
}
