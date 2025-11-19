#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

scan_points = None

def callback(scan):
    global scan_points
    
    # Original angles
    angles_rad = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges_m = np.array(scan.ranges)
    ranges_m[~np.isfinite(ranges_m)] = 0  # remove NaN/inf

    # Remove zero distance values
    valid_mask = ranges_m > 0
    ranges_m = ranges_m[valid_mask]
    angles_rad = angles_rad[valid_mask]

    # Convert meters → millimeters
    ranges_mm = ranges_m * 1000.0

    # ===== Filter: keep 120–160 mm =====
    min_mm = 120
    max_mm = 160
    mask = (ranges_mm >= min_mm) & (ranges_mm <= max_mm)
    ranges_mm = ranges_mm[mask]
    angles_rad = angles_rad[mask]

    # Convert to Cartesian (mm)
    x = ranges_mm * np.cos(angles_rad)
    y = ranges_mm * np.sin(angles_rad)

    # Rotate -90° (clockwise)
    theta = -np.pi / 2
    x_rot = x * np.cos(theta) - y * np.sin(theta)
    y_rot = x * np.sin(theta) + y * np.cos(theta)

    # Store x, y, and distance_mm
    if x_rot.size > 0:
        scan_points = np.vstack((x_rot, y_rot, ranges_mm))


def update_plot():
    global scan_points
    plt.clf()

    if scan_points is not None:
        x_vals = scan_points[0]
        y_vals = scan_points[1]
        dist_vals = scan_points[2]

        # Plot + color by distance
        plt.scatter(x_vals, y_vals, c=dist_vals, cmap='viridis', s=25)
        plt.plot(x_vals, y_vals, 'r-', alpha=0.6)

    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Rail Surface Scan (120–160 mm Range)")

    # ===== Tight, clear zoom =====
    plt.xlim(-40, 40)      # horizontal width
    plt.ylim(110, 170)     # around your detection zone

    # Grid
    plt.grid(True, linestyle='--', alpha=0.5)

    # Colorbar
    if scan_points is not None:
        plt.colorbar(label='Distance (mm)')

    plt.pause(0.05)


if __name__ == '__main__':
    rospy.init_node('trackeye_rplidar')
    rospy.Subscriber('/scan', LaserScan, callback)

    plt.ion()
    fig = plt.figure(figsize=(6, 6))

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        update_plot()
        rate.sleep()
