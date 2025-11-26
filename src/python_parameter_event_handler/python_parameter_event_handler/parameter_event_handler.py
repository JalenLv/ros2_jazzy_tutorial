import rclpy
from rclpy.node import Node
import rclpy.parameter

from rclpy.parameter_event_handler import ParameterEventHandler

class SampleNodeWithParameters(Node):
    def __init__(self):
        super().__init__("node_with_parameters")

        self.declare_parameter("an_int_param", 0)

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

    def callback(self, p: rclpy.parameter.Parameter) -> None:
        self.get_logger().info(
            "Received an update to parameter: {}: {}".format(
                p.name, rclpy.parameter.parameter_value_to_python(p.value)
            )
        )

def main():
    rclpy.init()
    node = SampleNodeWithParameters()
    rclpy.spin(node)
    rclpy.shutdown()
