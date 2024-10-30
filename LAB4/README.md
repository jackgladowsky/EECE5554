Lab 4 Navigation with IMU and Magnetometer

Steps to run:
1. Clone the project
2. Move the analysis and data folders to a seperate folder
3. Go into LAB4 and run ```colcon build``` to build the workspace
4. Run ```source install/setup.bash``` to source the workspace
5. Run ``` ls /dev/tty* ``` and to find what ports your sensors are connected too
6. Run ```ros2 launch sensors_launch sensors_launch.py gps_port:=//dev/ttyUSBX imu_port:=/dev/ttyUSBX``` and replace the X with the port number.