#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node {
public:
    explicit StatePublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("state_publisher", options)
    {
        // Create a publisher to tell robot_state_publisher the JointState information
        // robot_state_publisher will deal with this transformation
        this->joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Create a broadcaster to tell the tf2 state information
        // This broadcaster will determine the position of coordinate system `axis` in coordinate system `odom`
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Starting state publisher");

        this->timer_ = this->create_wall_timer(33ms, std::bind(&StatePublisher::publish, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state variables
    // `degree` means 0.5 degrees
    const double degree = M_PI / 360.0;
    double tilt = 0.0;
    double swivel = 0.0;
    double angle = 0.0;
    double height = 0.0;
    double tinc = degree;
    double hinc = 0.005;

    void publish() {
        // Create msgs
        geometry_msgs::msg::TransformStamped t;
        sensor_msgs::msg::JointState joint_state;

        // Add time stamp
        joint_state.header.stamp = this->get_clock()->now();
        // Specify joints' name which are defined in the r2d2.urdf.xml and their content
        joint_state.name = { "swivel", "tilt", "periscope" };
        joint_state.position = { this->swivel, this->tilt, this->height };

        // Add time stamp
        t.header.stamp = this->get_clock()->now();
        // Specify the father and child frame
        t.header.frame_id = "odom";
        t.child_frame_id = "axis";
        // Add translation change
        t.transform.translation.x = std::cos(this->angle) * 2;
        t.transform.translation.y = std::sin(this->angle) * 2;
        t.transform.translation.z = 0.7;
        // Add rotation change
        tf2::Quaternion q;
        q.setRPY(0, 0, this->angle + M_PI / 2);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Update the robot states
        this->tilt += this->tinc;
        if (this->tilt < -0.5 || this->tilt > 0.0) {
            this->tinc *= -1;
        }

        this->height += this->hinc;
        if (this->height > 0.2 || this->height < 0.0) {
            this->hinc *= -1;
        }

        this->swivel += this->degree;
        this->angle += this->degree;

        // Send message
        this->broadcaster_->sendTransform(t);
        this->joint_pub_->publish(joint_state);

        RCLCPP_INFO(this->get_logger(), "Publishing joint state");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}