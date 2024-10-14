from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the IMU'
        ),
        Node(
            package='imu_driver',
            executable='driver',
            name='imu_driver',
            parameters=[
                {'port': LaunchConfiguration('port')}
            ]
        )
    ])