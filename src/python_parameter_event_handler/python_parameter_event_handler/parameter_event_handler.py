import rclpy
from rclpy.node import Node
import rclpy.parameter
from rcl_interfaces.msg import Parameter, ParameterEvent

from rclpy.parameter_event_handler import ParameterEventHandler

class SampleNodeWithParameters(Node):
    def __init__(self):
        super().__init__("node_with_parameters")

        self.declare_parameter("an_int_param", 0)
        self.declare_parameter("another_double_param", 0.0)

        self.handler = ParameterEventHandler(self)

        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="an_int_param",
            node_name=self.get_name(),
            callback=self.callback,
        )

        self.handler.add_parameter_callback(
            "a_double_param",
            "parameter_blackboard",
            self.callback,
        )

        self.event_callback_handle = self.handler.add_parameter_event_callback(
            self.event_callback
        )

    def callback(self, p: Parameter) -> None:
        self.get_logger().info(
            "Received an update to parameter: {}: {}".format(
                p.name, rclpy.parameter.parameter_value_to_python(p.value)
            )
        )

    def event_callback(self, parameter_event: ParameterEvent):
        self.get_logger().info(f"Type: {type(parameter_event)}")
        self.get_logger().info(f"Type: {type(parameter_event.changed_parameters)}")
        self.get_logger().info(f"Type: {type(parameter_event.changed_parameters[0])}")

        self.get_logger().info(f"Received parameter event from node {parameter_event.node}")

        p: Parameter
        for p in parameter_event.changed_parameters:
            self.get_logger().info(
                "Inside event: {} changed to: {}".format(
                    p.name, rclpy.parameter.parameter_value_to_python(p.value)
                )
            )

def main():
    rclpy.init()
    node = SampleNodeWithParameters()
    rclpy.spin(node)
    rclpy.shutdown()
