from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments for port configuration
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='Port for the GPS sensor'
    )
    
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Port for the IMU sensor'
    )

    # Define the nodes
    gps_node = Node(
        package='gps_driver',
        executable='gps_driver',
        name='gps_driver',
        parameters=[{'port': LaunchConfiguration('gps_port')}]
    )

    imu_node = Node(
        package='imu_driver',
        executable='imu_driver',
        name='imu_driver',
        parameters=[{'port': LaunchConfiguration('imu_port')}]
    )

    # Create and return the launch description
    return LaunchDescription([
        gps_port_arg,
        imu_port_arg,
        gps_node,
        imu_node
    ])