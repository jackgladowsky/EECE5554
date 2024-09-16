import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class PA1Subscriber(Node):
    def __init__(self):
        super().__init__('PA1_Subscriber')
        self.subscription = self.create_sibscription(
            String,
            'topic',
            self.listener_callback, 
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Subscribed Message: {msg.data}")   

def main(args=None):
    rclpy.init(args=args)
    subscriber = PA1Subscriber()
    rclpy.spin(subscriber)

if __name__ == '__main__':
    main()