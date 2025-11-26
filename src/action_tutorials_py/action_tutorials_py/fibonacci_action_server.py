import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from custom_action_interfaces.action import Fibonacci

import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__("fibonacci_action_server")
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "fibonacci",
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        request: Fibonacci.Request = goal_handle.request
        for i in range(1, request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )
            self.get_logger().info("Feedback: {}".format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main():
    rclpy.init()

    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
