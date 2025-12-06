#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"

class FrameListener : public rclcpp::Node {
public:
    explicit FrameListener() : Node("turtle_tf2_frame_listener"),
                               turtle_spawning_service_ready_(false),
                               turtle_spawned_(false)
    {
        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter("target_frame", "turtle1");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a client to spawn a turtle
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        // Create turtle2 velocity publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle2/cmd_vel", 1
        );

        // Call on_timer function every second
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250), [this] () { this->on_timer(); }
        );
    }

private:
    // Boolean values to store the information
    // if the service for spawning turtle is available
    bool turtle_spawning_service_ready_;
    // if the turtle was successfully spawned
    bool turtle_spawned_;

    std::string target_frame_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;

    void on_timer() {
        // Store frame names in variables that will be used
        // to compute transformations
        std::string fromFrameRel = target_frame_;
        std::string toFrameRel = "turtle2";

        if (this->turtle_spawning_service_ready_) {
            if (this->turtle_spawned_) {
                geometry_msgs::msg::TransformStamped t;

                // Look up for the transformation between target_frame and turtle2 frames
                // and send velocity commands for turtle2 to reach target_frame
                try {
                    t = tf_buffer_->lookupTransform(
                        toFrameRel, fromFrameRel,
                        this->get_clock()->now(),
                        // rclcpp::Duration::from_nanoseconds(50)
                        std::chrono::milliseconds(50)
                    );
                } catch(const tf2::TransformException &e) {
                    RCLCPP_WARN(this->get_logger(),
                        "Could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), e.what()
                    );
                    return;
                }
                
                geometry_msgs::msg::Twist msg;
                static const double scaleRotationRate = 1.0;
                msg.angular.z = scaleRotationRate * std::atan2(
                    t.transform.translation.y,
                    t.transform.translation.x
                );

                static const double scaleForwardSpeed = 0.5;
                msg.linear.x = scaleForwardSpeed * std::sqrt(
                    std::pow(t.transform.translation.x, 2) +
                    std::pow(t.transform.translation.y, 2)
                );

                publisher_->publish(msg);
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                this->turtle_spawned_ = true;
            }
        } else {
            // Check if the service is ready
            if (spawner_->service_is_ready()) {
                // Init request with turtle name and coordinates
                // Note that x, y and theta are defined as floats in tyrtlesim/srv/Spawn
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = "turtle2";

                // Call request
                auto result = spawner_->async_send_request(
                    request, [this] (ServiceResponseFuture future) { this->response_received_callback(future); }
                );
            } else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }

    void response_received_callback(ServiceResponseFuture future) {
        auto result = future.get();
        if (result->name == "turtle2") {
            this->turtle_spawning_service_ready_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}
