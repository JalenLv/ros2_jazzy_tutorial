#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <memory>

void add(
    const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,
    std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response> response
) {
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Incoming request\na: %ld b: %ld c: %ld",
        request->a, request->b, request->c
    );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Sending back response: [%ld]", static_cast<long int>(response->sum)
    );
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("add_three_ints_server");

    auto service = node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
