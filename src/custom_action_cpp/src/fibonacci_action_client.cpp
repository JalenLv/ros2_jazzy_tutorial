#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "custom_action_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom_action_cpp {

class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = custom_action_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
    : Node("fibonacci_action_client", options) {
        client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        auto timer_callback = [this] () { return send_goal(); };
        timer_ = create_wall_timer(std::chrono::milliseconds(500), timer_callback);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal() {
        using namespace std::placeholders;

        // Cancel the timer so the goal is sent only once
        timer_->cancel();

        if (!client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO_STREAM(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this] (const GoalHandleFibonacci::SharedPtr &goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };
        send_goal_options.feedback_callback = [this] (
            GoalHandleFibonacci::SharedPtr goal_handle,
            const std::shared_ptr<const Fibonacci::Feedback> feedback
        ) {
            (void)goal_handle;
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for (const auto &number : feedback->partial_sequence) {
                ss << number << " ";
            }
            RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
        };
        send_goal_options.result_callback = [this] (const GoalHandleFibonacci::WrappedResult &result) {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown result code");
                break;
            }

            std::stringstream ss;
            ss << "Result received: ";
            for (const auto &number: result.result->sequence) {
                ss << number << " ";
            }
            RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
            rclcpp::shutdown();
        };
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
}; // class FibonacciActionClient

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionClient);
