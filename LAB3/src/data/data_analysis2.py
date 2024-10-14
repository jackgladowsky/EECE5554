import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

def read_rosbag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    data = []
    while reader.has_next():
        (topic, msg, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(msg, msg_type)
        data.append((topic, msg_deserialized, t))
    return data

def analyze_imu_data(bag_path):
    # Read the ROS2 bag
    data = read_rosbag(bag_path)

    # Initialize lists to store data
    times = []
    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []
    mag_x, mag_y, mag_z = [], [], []

    # Extract data from the bag
    for topic, msg, t in data:
        if topic == '/imu':
            times.append(t)
            # Access IMU data
            imu_data = msg.imu
            accel_x.append(imu_data.linear_acceleration.x)
            accel_y.append(imu_data.linear_acceleration.y)
            accel_z.append(imu_data.linear_acceleration.z)
            gyro_x.append(imu_data.angular_velocity.x)
            gyro_y.append(imu_data.angular_velocity.y)
            gyro_z.append(imu_data.angular_velocity.z)
            # Access magnetometer data
            mag_data = msg.mag_field
            mag_x.append(mag_data.magnetic_field.x)
            mag_y.append(mag_data.magnetic_field.y)
            mag_z.append(mag_data.magnetic_field.z)

    # Convert lists to numpy arrays
    times = np.array(times)
    accel_x, accel_y, accel_z = map(np.array, [accel_x, accel_y, accel_z])
    gyro_x, gyro_y, gyro_z = map(np.array, [gyro_x, gyro_y, gyro_z])
    mag_x, mag_y, mag_z = map(np.array, [mag_x, mag_y, mag_z])

    # Scale magnetometer data
    mag_scale = 1e5  # Convert to nanoteslas
    mag_x *= mag_scale
    mag_y *= mag_scale
    mag_z *= mag_scale

    # Analyze and plot data
    analyze_sensor(times, accel_x, accel_y, accel_z, "Accelerometer", "m/s^2")
    analyze_sensor(times, gyro_x, gyro_y, gyro_z, "Gyroscope", "rad/s")
    analyze_sensor(times, mag_x, mag_y, mag_z, "Magnetometer", "nT")

def analyze_sensor(times, x, y, z, sensor_name, unit):
    print(f"\n{sensor_name} Statistics:")
    for axis, data in zip(['X', 'Y', 'Z'], [x, y, z]):
        mean = np.mean(data)
        std = np.std(data)
        print(f"  {axis}-axis: Mean = {mean:.6f}, Std Dev = {std:.6f}")

    plot_time_series(times, x, y, z, sensor_name, unit)
    plot_histogram(x, y, z, sensor_name, unit)

def plot_time_series(times, x, y, z, title, unit):
    plt.figure(figsize=(12, 6))
    plt.plot(times, x, label='X')
    plt.plot(times, y, label='Y')
    plt.plot(times, z, label='Z')
    plt.title(f"{title} Time Series")
    plt.xlabel("Time (nanoseconds)")
    plt.ylabel(f"Value ({unit})")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"{title.lower()}_time_series.png")
    plt.close()

def plot_histogram(x, y, z, title, unit):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    
    for ax, data, axis in zip([ax1, ax2, ax3], [x, y, z], ['X', 'Y', 'Z']):
        mu, std = norm.fit(data)
        ax.hist(data, bins=50, density=True, alpha=0.7)
        xmin, xmax = ax.get_xlim()
        x_range = np.linspace(xmin, xmax, 100)
        p = norm.pdf(x_range, mu, std)
        ax.plot(x_range, p, 'k', linewidth=2)
        ax.set_title(f"{axis}-axis")
        ax.set_xlabel(f"Value ({unit})")
        ax.set_ylabel("Frequency")
        if title == "Magnetometer":
            ax.set_yscale('log')  # Use log scale for magnetometer y-axis

    plt.suptitle(f"{title} Histogram")
    plt.tight_layout()
    plt.savefig(f"{title.lower()}_histogram.png")
    plt.close()

if __name__ == "__main__":
    bag_path = "./rosbag2_2024_10_14-15_00_39"  # Replace with your actual bag file path
    analyze_imu_data(bag_path)