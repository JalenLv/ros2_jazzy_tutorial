import sys

from tutorial_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.cli = self.create_client(AddThreeInts, "add_three_ints")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            # The while loop in the constructor checks if
            # a service matching the type and name of the client
            # is available once a second.
            self.get_logger().info("service not available, waiting again...")
        self.req = AddThreeInts.Request()

    def send_request(self, a: int, b: int, c: int):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        return self.cli.call_async(self.req)
        
def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    rclpy.spin_until_future_complete(minimal_client, future)

    response: AddThreeInts.Response = future.result()
    minimal_client.get_logger().info(
        "Result of add_two_ints: for {} + {} + {} = {}".format(
                int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum
            )
    )

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
