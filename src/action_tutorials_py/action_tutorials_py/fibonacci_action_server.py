import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from custom_action_interfaces.action import Fibonacci

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

        sequence = [0, 1]
        request: Fibonacci.Request = goal_handle.request
        for i in range(1, request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result

def main():
    rclpy.init()

    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
