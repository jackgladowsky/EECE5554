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
        data.append(msg_deserialized)

    return data

def distance_formula(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def analyze_stationary_data(data):
    utm_easting = [msg.utm_easting for msg in data]
    utm_northing = [msg.utm_northing for msg in data]
    alt = [msg.altitude for msg in data]

    # Calculate statistics
    easting_mean, easting_std = np.mean(utm_easting), np.std(utm_easting)
    northing_mean, northing_std = np.mean(utm_northing), np.std(utm_northing)
    alt_mean, alt_std = np.mean(alt), np.std(alt)

    # Calculate 2D error
    errors_2d = [distance_formula(easting_mean, northing_mean, e, n) for e, n in zip(utm_easting, utm_northing)]
    mean_error_2d = np.mean(errors_2d)
    std_error_2d = np.std(errors_2d)

    # Plot scatter
    plt.figure(figsize=(10, 10))
    scatter = plt.scatter(utm_easting, utm_northing, c=alt, cmap='viridis')
    plt.colorbar(scatter, label='Altitude (m)')
    plt.title('GPS Stationary Data (UTM)')
    plt.xlabel('UTM Easting (m)')
    plt.ylabel('UTM Northing (m)')
    plt.savefig('output/stationary_scatter_utm.png')
    plt.close()

    # Plot histograms
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
    ax1.hist(errors_2d, bins=30, density=True, alpha=0.7)
    ax1.set_title('2D Error Distribution')
    ax1.set_xlabel('Error (m)')
    ax1.set_ylabel('Density')

    # Fit a normal distribution to the errors
    mu, sigma = norm.fit(errors_2d)
    x = np.linspace(0, max(errors_2d), 100)
    p = norm.pdf(x, mu, sigma)
    ax1.plot(x, p, 'k', linewidth=2)

    ax2.hist(alt, bins=30, density=True, alpha=0.7)
    ax2.set_title('Altitude Distribution')
    ax2.set_xlabel('Altitude (m)')
    ax2.set_ylabel('Density')

    plt.savefig('output/stationary_error_distribution_utm.png')
    plt.close()

    return (easting_mean, easting_std, northing_mean, northing_std, 
            alt_mean, alt_std, mean_error_2d, std_error_2d)

def analyze_walking_data(data):
    utm_easting = [msg.utm_easting for msg in data]
    utm_northing = [msg.utm_northing for msg in data]
    alt = [msg.altitude for msg in data]
    
    # Calculate distances between consecutive points
    distances = [distance_formula(utm_easting[i], utm_northing[i], 
                                    utm_easting[i+1], utm_northing[i+1]) 
                 for i in range(len(utm_easting)-1)]
    total_distance = np.sum(distances)
    
    # Calculate speeds between consecutive points (assuming 1 second intervals)
    speeds = [d * 3.6 for d in distances]  # Convert m/s to km/h
    
    # Plot UTM trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(utm_easting, utm_northing)
    plt.title('GPS Walking Data (UTM)')
    plt.xlabel('UTM Easting (m)')
    plt.ylabel('UTM Northing (m)')
    plt.savefig('output/walking_utm_trajectory.png')
    plt.close()
    
    # Plot speed over time
    plt.figure(figsize=(10, 5))
    plt.plot(speeds)
    plt.title('Walking Speed')
    plt.xlabel('Sample')
    plt.ylabel('Speed (km/h)')
    plt.savefig('output/walking_speed.png')
    plt.close()

    # Plot altitude profile
    plt.figure(figsize=(10, 5))
    plt.plot(alt)
    plt.title('Altitude Profile')
    plt.xlabel('Sample')
    plt.ylabel('Altitude (m)')
    plt.savefig('output/walking_altitude_profile.png')
    plt.close()

    return total_distance, np.mean(speeds), np.std(speeds)

def analyze_walking_error(data):
    utm_easting = [msg.utm_easting for msg in data]
    utm_northing = [msg.utm_northing for msg in data]

    # Linear regression
    coeffs = np.polyfit(utm_easting, utm_northing, 1)
    poly = np.poly1d(coeffs)

    # Calculate distance of each point to the fitted line
    distances = np.abs(utm_northing - poly(utm_easting)) / np.sqrt(1 + coeffs[0]**2)

    plt.figure(figsize=(10, 10))
    plt.scatter(utm_easting, utm_northing, label='Data')
    plt.plot(utm_easting, poly(utm_easting), color='r', label='Fitted Line')
    plt.title('Walking Data with Linear Fit (UTM)')
    plt.xlabel('UTM Easting (m)')
    plt.ylabel('UTM Northing (m)')
    plt.legend()
    plt.savefig('output/walking_linear_fit_utm.png')
    plt.close()

    # Plot error distribution
    plt.figure(figsize=(10, 5))
    plt.hist(distances, bins=30, density=True, alpha=0.7)
    plt.title('Walking Error Distribution')
    plt.xlabel('Error (m)')
    plt.ylabel('Density')
    plt.savefig('output/walking_error_distribution_utm.png')
    plt.close()

    return np.mean(distances), np.std(distances)

# Main execution
stationary_data = read_rosbag('./data/stationary_data2.bag')
walking_data = read_rosbag('./data/walking_data3.bag')

# Analyze stationary data
stats = analyze_stationary_data(stationary_data)
print(f"UTM Easting: mean = {stats[0]:.2f} m, std = {stats[1]:.2f} m")
print(f"UTM Northing: mean = {stats[2]:.2f} m, std = {stats[3]:.2f} m")
print(f"Altitude: mean = {stats[4]:.2f} m, std = {stats[5]:.2f} m")
print(f"2D Error: mean = {stats[6]:.2f} m, std = {stats[7]:.2f} m")

# Analyze walking data
distance, mean_speed, std_speed = analyze_walking_data(walking_data)
print(f"Total walking distance: {distance:.2f} meters")
print(f"Average speed: {mean_speed:.2f} km/h")
print(f"Speed standard deviation: {std_speed:.2f} km/h")

# Analyze walking error
mean_error, std_error = analyze_walking_error(walking_data)
print(f"Mean distance from line: {mean_error:.2f} meters")
print(f"Standard deviation of distance from line: {std_error:.2f} meters")