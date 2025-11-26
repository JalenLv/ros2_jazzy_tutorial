import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_action_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__("fibonacci_action_client")
        self._action_client = ActionClient(
            self,
            Fibonacci,
            "fibonacci",
        )
    
    def send_goal(self, order: int):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        while not self._action_client.wait_for_server():
            self.get_logger().info("Waiting for action server...")

        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()

    fibonacci_action_client = FibonacciActionClient()
    future = fibonacci_action_client.send_goal(10)
    rclpy.spin_until_future_complete(fibonacci_action_client, future)

    fibonacci_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
