import rclpy
from rclpy.node import Node
import serial
import utm
from gps_msg.msg import GPSmsg
from std_msgs.msg import Header

class GPSDriver(Node):

    def __init__(self):
        super().__init__('gps_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 4800) # 9600???

        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.publisher = self.create_publisher(GPSmsg, 'gps', 10)
        self.timer = self.create_timer(.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('$GPGGA'):
                msg = self.parse_gpgga(line)
                if msg:
                    self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {str(e)}')

    def parse_gpgga(self, gpgga_string):
        parts = gpgga_string.split(',')
        if len(parts) < 15:
            return None

        try:
            msg = GPSmsg()
            msg.header = Header()

            # Parse time stamp
            # 023540.236
            gps_time = parts[1]
            msg.header.stamp.sec = int(gps_time[0:6])
            msg.header.stamp.nanosec = int(gps_time[7:])

            #msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "GPS1_Frame"

            # Parse latitude
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60
            if parts[3] == 'S':
                lat = -lat
            msg.latitude = lat

            # Parse longitude
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60
            if parts[5] == 'W':
                lon = -lon
            msg.longitude = lon

            # Parse altitude
            msg.altitude = float(parts[9])

            # Convert to UTM
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            msg.utm_easting = easting
            msg.utm_northing = northing
            msg.zone = zone_number
            msg.letter = zone_letter

            return msg
        except Exception as e:
            self.get_logger().error(f'Error parsing GPGGA string: {str(e)}')
            return None
        
def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPSDriver()
    rclpy.spin(gps_driver)
    gps_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()