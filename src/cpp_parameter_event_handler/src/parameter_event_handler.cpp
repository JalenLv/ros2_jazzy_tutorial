#include <memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node {
public:
    SampleNodeWithParameters() : Node("node_with_parameters") {
        this->declare_parameter<int>("an_int_param", 0);
        this->declare_parameter<double>("another_double_param", 0.0);

        // Create a parameter subscriber that can be used to monitor parameter changes
        // (for this node's parameters as well as other nodes' parameters)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Set a callback for this node's interget parameter, "an_int_param"
        auto cb_lambda = [this] (const rclcpp::Parameter &p) { cb(p); };
        cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb_lambda);

        // Now, add a callback to monitor any changes to the remote node's parameter.
        // In this case, we supply the remote node name.
        auto cb2_lambda = [this] (const rclcpp::Parameter &p) { cb2(p); };
        auto remote_node_name = std::string("parameter_blackboard");
        auto remote_param_name = std::string("a_double_param");
        cb2_handle_ = param_subscriber_->add_parameter_callback(
            remote_param_name, cb2_lambda, remote_node_name
        );

        auto event_cb_lambda =
            [this] (const rcl_interfaces::msg::ParameterEvent &param_event) { event_cb(param_event); };
        event_cb_handle_ = param_subscriber_->add_parameter_event_callback(event_cb_lambda);
    }

private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb2_handle_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> event_cb_handle_;

    void cb(const rclcpp::Parameter &p) {
        RCLCPP_INFO(this->get_logger(),
            "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int()
        );
    }

    void cb2(const rclcpp::Parameter &p) {
        RCLCPP_INFO(this->get_logger(),
            "cb2: Received an update to parameter \"%s\" of type %s: \"%.02lf\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_double()
        );
    }

    void event_cb(const rcl_interfaces::msg::ParameterEvent &param_event) {
        RCLCPP_INFO(this->get_logger(),
            "Received parameter event from node \"%s\"",
            param_event.node.c_str()
        );
        for (const auto &p : param_event.changed_parameters) {
            RCLCPP_INFO(this->get_logger(),
                "Inside event: \"%s\" changed to %s",
                p.name.c_str(),
                rclcpp::Parameter::from_parameter_msg(p).value_to_string().c_str()
            );
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
    rclcpp::shutdown();
    return 0;
}