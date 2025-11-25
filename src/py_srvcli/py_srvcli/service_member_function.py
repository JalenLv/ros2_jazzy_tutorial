from example_interfaces.srv import AddTwoInts
from example_interfaces.srv import AddTwoInts_Request
from example_interfaces.srv import AddTwoInts_Response

import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__("minimal_service")
        self.srv = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request: AddTwoInts_Request, response: AddTwoInts_Response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Incoming request\na: {request.a} b: {request.b}")

        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == "__name__":
    main()
