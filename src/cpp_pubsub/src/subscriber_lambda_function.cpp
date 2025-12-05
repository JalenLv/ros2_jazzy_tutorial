#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        auto topic_callback =
            [this](const tutorial_interfaces::msg::Num &msg) -> void {
                RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
            };
        subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
            "topic", 10, topic_callback
        );
    }

private:
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
