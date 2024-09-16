import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class PA1Publisher(Node):

    def __init__(self):
        super().__init__('PA1_Publisher') # Set the name of the Node

        # create a publisher to publish a string to the 'topic' topic
        # saves the last 10 msgs sent through publisher
        self.publisher = self.create_publisher(String, 'topic', 10) 
        timer_period = 0.5

        # creates a timer that calls the publisher every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    # timer callback function
    def timer_callback(self):
        msg = String()
        # sets the data to be published
        msg.data = f'EECE5554 PA1: {self.count}'
        # publish the data
        self.publisher.publish(msg)
        # log the published data to the terminal
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args) # init the ROS2 python packages

    publisher = PA1Publisher() # create the publisher item

    rclpy.spin(publisher) # spin the publisher item 

if __name__ == '__main__':
    main()
