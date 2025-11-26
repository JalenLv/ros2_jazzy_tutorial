import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

from custom_action_interfaces.action import Fibonacci
from custom_action_interfaces.action import Fibonacci_GetResult_Response

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

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        get_result_response: Fibonacci_GetResult_Response = future.result()
        result: Fibonacci.Result = get_result_response.result
        self.get_logger().info("Result: {}".format(result.sequence))
        rclpy.shutdown()

def main():
    rclpy.init()

    fibonacci_action_client = FibonacciActionClient()
    fibonacci_action_client.send_goal(10)
    rclpy.spin(fibonacci_action_client)

if __name__ == "__main__":
    main()
