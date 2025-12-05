#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace palomino {

class VincentDriver : public rclcpp::Node {
public:
    VincentDriver() : Node("vincent_driver") {
        RCLCPP_INFO(this->get_logger(), "Hello from VincentDriver node!");
    }
}; // class VincentDriver

} // namespace palomino

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<palomino::VincentDriver>());
    rclcpp::shutdown();
    return 0;
}
