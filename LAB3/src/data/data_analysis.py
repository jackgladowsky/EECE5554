import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from transforms3d.euler import quat2euler

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
    roll, pitch, yaw = [], [], []

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
            # Convert quaternion to Euler angles
            q = imu_data.orientation
            euler = quat2euler([q.w, q.x, q.y, q.z])
            roll.append(euler[0])
            pitch.append(euler[1])
            yaw.append(euler[2])

    # Convert lists to numpy arrays
    times = np.array(times)
    accel_x, accel_y, accel_z = map(np.array, [accel_x, accel_y, accel_z])
    gyro_x, gyro_y, gyro_z = map(np.array, [gyro_x, gyro_y, gyro_z])
    mag_x, mag_y, mag_z = map(np.array, [mag_x, mag_y, mag_z])
    roll, pitch, yaw = map(np.array, [roll, pitch, yaw])

    # Plot time series
    plot_time_series(times, accel_x, accel_y, accel_z, "Accelerometer")
    plot_time_series(times, gyro_x, gyro_y, gyro_z, "Gyroscope")
    plot_time_series(times, mag_x, mag_y, mag_z, "Magnetometer")
    plot_time_series(times, roll, pitch, yaw, "Euler Angles")

    # Calculate and print statistics
    print_statistics(accel_x, accel_y, accel_z, "Accelerometer")
    print_statistics(gyro_x, gyro_y, gyro_z, "Gyroscope")
    print_statistics(mag_x, mag_y, mag_z, "Magnetometer")
    print_statistics(roll, pitch, yaw, "Euler Angles")

    # Plot histograms
    plot_histogram(accel_x, accel_y, accel_z, "Accelerometer")
    plot_histogram(gyro_x, gyro_y, gyro_z, "Gyroscope")
    plot_histogram(mag_x, mag_y, mag_z, "Magnetometer")
    plot_histogram(roll, pitch, yaw, "Euler Angles")

def plot_time_series(times, x, y, z, title):
    axes = ['X', 'Y', 'Z'] if title != "Euler Angles" else ['Roll', 'Pitch', 'Yaw']
    data = [x, y, z]
    
    fig, axs = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"{title} Time Series", fontsize=16)
    
    for i, (ax, axis, values) in enumerate(zip(axs, axes, data)):
        ax.plot(times, values)
        ax.set_title(f"{axis}")
        ax.set_ylabel(f"{axis} Value")
        ax.grid(True)
    
    axs[-1].set_xlabel("Time (nanoseconds)")
    plt.tight_layout()
    plt.savefig(f"{title.lower().replace(' ', '_')}_time_series.png")
    plt.close()

def print_statistics(x, y, z, title):
    print(f"\n{title} Statistics:")
    axes = ['X', 'Y', 'Z'] if title != "Euler Angles" else ['Roll', 'Pitch', 'Yaw']
    for axis, data in zip(axes, [x, y, z]):
        mean = np.mean(data)
        std = np.std(data)
        print(f"  {axis}: Mean = {mean:.9f}, Std Dev = {std:.6f}")

def plot_histogram(x, y, z, title):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    axes = ['X', 'Y', 'Z'] if title != "Euler Angles" else ['Roll', 'Pitch', 'Yaw']
    
    for ax, data, axis in zip([ax1, ax2, ax3], [x, y, z], axes):
        mu, std = norm.fit(data)
        ax.hist(data, bins=50, density=True, alpha=0.7)
        xmin, xmax = ax.get_xlim()
        x = np.linspace(xmin, xmax, 100)
        p = norm.pdf(x, mu, std)
        ax.plot(x, p, 'k', linewidth=2)
        ax.set_title(f"{axis}")
        ax.set_xlabel("Value")
        ax.set_ylabel("Frequency")
    
    plt.suptitle(f"{title} Histogram")
    plt.tight_layout()
    plt.savefig(f"{title.lower().replace(' ', '_')}_histogram.png")
    plt.close()

if __name__ == "__main__":
    bag_path = "./stationary15_1"
    analyze_imu_data(bag_path)