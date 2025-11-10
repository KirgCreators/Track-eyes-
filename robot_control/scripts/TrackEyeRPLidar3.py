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
    ranges = np.array(scan.ranges)
    ranges[~np.isfinite(ranges)] = 0

    # Remove zero ranges
    nonzero_mask = ranges > 0
    angles_rad = angles_rad[nonzero_mask]
    ranges = ranges[nonzero_mask]

    # Convert to Cartesian coordinates
    x = ranges * np.cos(angles_rad)
    y = ranges * np.sin(angles_rad)

    # --- ROTATE coordinates so "motor down" is aligned with -Y, clockwise ---
    # -90 deg rotation (clockwise)
    theta = -np.pi / 2
    x_rot = x * np.cos(theta) - y * np.sin(theta)
    y_rot = x * np.sin(theta) + y * np.cos(theta)

    if x_rot.size > 0:
        scan_points = np.vstack((x_rot, y_rot))


def update_plot():
    global scan_points
    plt.clf()
    if scan_points is not None:
        plt.scatter(scan_points[0], scan_points[1], c='b', s=5)
        plt.plot(scan_points[0], scan_points[1], 'r-')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Rail Surface Scan")
    plt.xlim(-2, 2)
    plt.ylim(0, 2)
    plt.pause(0.05)

if __name__ == '__main__':
    rospy.init_node('trackeye_rplidar')
    rospy.Subscriber('/scan', LaserScan, callback)

    plt.ion()
    fig = plt.figure()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        update_plot()
        rate.sleep()