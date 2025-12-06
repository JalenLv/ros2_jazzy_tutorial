#include <memory>
#include "rclcpp/rclcpp.hpp"

class SimpleNode : public rclcpp::Node {
public:
    SimpleNode(const std::string &node_name) : Node(node_name) {}
};

void node_info(rclcpp::Node::SharedPtr node) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Node name: " << node->get_name());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SimpleNode>("simple_node");
    node_info(node);

    rclcpp::shutdown();
    return 0;
}
