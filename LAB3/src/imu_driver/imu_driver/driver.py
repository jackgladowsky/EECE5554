import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import Imu, MagneticField
from imu_msg.msg import IMUmsg

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.publisher = self.create_publisher(IMUmsg, 'imu', 10)
        self.timer = self.create_timer(0.025, self.timer_callback)  # 40 Hz
        
        # Configure IMU to output at 40 Hz
        self.ser.write(b'$VNWRG,07,40*59\r\n')  # Replace XX with actual checksum
        
    def timer_callback(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            self.get_logger().info(f"Raw IMU data: {line}")  # Log raw data
            
            if line.startswith('$VNYMR'):
                # Split the string
                data = line.split('*')[0].split(',')
                
                if len(data) == 13:  # Confirm we have the expected number of fields
                    msg = IMUmsg()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "IMU1_Frame"
                    
                    # Parse YPR, accel, gyro, and mag data
                    yaw, pitch, roll = map(float, data[1:4])
                    mag = list(map(float, data[4:7]))
                    accel = list(map(float, data[7:10]))
                    gyro = list(map(float, data[10:13]))
                    
                    # Convert degrees to radians for Euler angles
                    yaw_rad = math.radians(yaw)
                    pitch_rad = math.radians(pitch)
                    roll_rad = math.radians(roll)
                    
                    # Convert Euler angles to quaternions
                    q = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                    
                    # Populate IMU message
                    msg.imu.orientation.x = q[0]
                    msg.imu.orientation.y = q[1]
                    msg.imu.orientation.z = q[2]
                    msg.imu.orientation.w = q[3]
                    
                    msg.imu.angular_velocity.x = gyro[0]
                    msg.imu.angular_velocity.y = gyro[1]
                    msg.imu.angular_velocity.z = gyro[2]
                    
                    # Accelerometer data is already in m/s^2
                    msg.imu.linear_acceleration.x = accel[0]
                    msg.imu.linear_acceleration.y = accel[1]
                    msg.imu.linear_acceleration.z = accel[2]
                    
                    # Populate MagneticField message (assuming values are in Gauss, convert to Tesla)
                    gauss_to_tesla = 1e-4
                    msg.mag_field.magnetic_field.x = mag[0] * gauss_to_tesla
                    msg.mag_field.magnetic_field.y = mag[1] * gauss_to_tesla
                    msg.mag_field.magnetic_field.z = mag[2] * gauss_to_tesla
                    
                    # Store raw IMU string
                    msg.raw_imu_string = line
                    
                    self.publisher.publish(msg)
                else:
                    self.get_logger().warn(f"Unexpected number of data fields: {len(data)}")
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        
        return q

def main(args=None):
    rclpy.init(args=args)
    imu_driver = IMUDriver()
    rclpy.spin(imu_driver)
    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()