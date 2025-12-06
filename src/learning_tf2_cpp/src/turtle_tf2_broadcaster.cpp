#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtlesim/msg/pose.hpp"

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher() : Node("turtle_tf2_frame_publisher") {
        // Declare and acquire `turtlename` parameter
        turtlename_ = this->declare_parameter("turtlename", "turtle");

        // Init the transform broadcaster
        // ? Should we dereference `this` ?
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscribe to a turtle{1}{2}/pose topic and call callback on each msg
        std::stringstream ss;
        ss << "/" << turtlename_ << "/pose";
        auto topic_name = ss.str();

        auto handle_turtle_pose_lambda = [this] (const turtlesim::msg::Pose::SharedPtr msg) {
            this->handle_turtle_pose(msg);
        };
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10,
            handle_turtle_pose_lambda
        );
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;

    void handle_turtle_pose(const turtlesim::msg::Pose::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;

        // Read msg content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtlename_;

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the msg and set the z coordinate to 0
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this is why set rotation in x and y to 0 and obtain
        // ratation in z axis from the msg
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
