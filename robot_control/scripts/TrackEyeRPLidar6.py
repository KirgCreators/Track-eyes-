#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

scan_points = None

def callback(scan):
    global scan_points
    
    # Original angles from LIDAR
    angles_rad = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges_m = np.array(scan.ranges)
    ranges_m[~np.isfinite(ranges_m)] = 0

    # Remove zero values
    nonzero = ranges_m > 0
    ranges_m = ranges_m[nonzero]
    angles_rad = angles_rad[nonzero]

    # Convert to centimeters
    ranges_cm = ranges_m * 100.0

    # Filter range: 10–20 cm
    min_cm = 10
    max_cm = 20
    mask = (ranges_cm >= min_cm) & (ranges_cm <= max_cm)
    ranges_cm = ranges_cm[mask]
    angles_rad = angles_rad[mask]

    # Convert to Cartesian (in cm)
    x = ranges_cm * np.cos(angles_rad)
    y = ranges_cm * np.sin(angles_rad)

    # Rotate 90° clockwise
    theta = -np.pi / 2
    x_rot = x * np.cos(theta) - y * np.sin(theta)
    y_rot = x * np.sin(theta) + y * np.cos(theta)

    if x_rot.size > 0:
        # Add distance as extra info
        scan_points = np.vstack((x_rot, y_rot, ranges_cm))


def update_plot():
    global scan_points
    plt.clf()

    if scan_points is not None:
        x_vals = scan_points[0]
        y_vals = scan_points[1]
        distances = scan_points[2]

        # Color points by distance (optional)
        plt.scatter(x_vals, y_vals, c=distances, cmap='viridis', s=20)
        plt.plot(x_vals, y_vals, 'r-', alpha=0.5)

    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.title("Rail Surface Scan (10–20 cm range)")
    
    # Axis limits
    plt.xlim(-20, 20)
    plt.ylim(0, 25)

    # Add grid
    plt.grid(True, linestyle='--', alpha=0.5)

    # Show colorbar for distance
    if scan_points is not None:
        plt.colorbar(label='Distance (cm)')

    plt.pause(0.05)


if __name__ == '__main__':
    rospy.init_node('trackeye_rplidar')
    rospy.Subscriber('/scan', LaserScan, callback)

    plt.ion()
    fig = plt.figure(figsize=(6,6))

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        update_plot()
        rate.sleep()
