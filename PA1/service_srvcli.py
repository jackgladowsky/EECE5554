from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class PA1Service(Node):
    def __init__(self):
        super().__init__('PA1_Service')
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.service_callback)

    def service_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request a: {request.a} b: {request.b}')
        return response
    
def main():
    rclpy.init()
    service = PA1Service()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()