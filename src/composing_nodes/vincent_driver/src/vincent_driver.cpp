#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace palomino {

class VincentDriver : public rclcpp::Node {
public:
    VincentDriver(const rclcpp::NodeOptions &options) : Node("vincent_driver", options) {
        RCLCPP_INFO(this->get_logger(), "Hello from VincentDriver node!");
    }
}; // class VincentDriver

} // namespace palomino

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver);
