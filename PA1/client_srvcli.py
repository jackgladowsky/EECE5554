import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class PA1Client(Node):
    def __init__(self):
        super().__init__('PA1_Client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        return self.client.call_async(self.request)
    
def main():
    rclpy.init()

    client = PA1Client()
    future = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(f'Result of adding two nums: {int(sys.argv[1])} + {int(sys.argv[2])} = {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()