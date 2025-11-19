#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

scan_points = None

def callback(scan):
    global scan_points
    # Use actual angles from the scan message (in radians)
    angles_rad = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)
    ranges[~np.isfinite(ranges)] = 0
    

    # Focus on one side (0 to 180 degrees)
    mask = (angles_rad >= 0) & (angles_rad <= np.pi)
    left_angles = angles_rad[mask]
    left_ranges = ranges[mask]
    nonzero_mask = left_ranges > 0
    left_angles = left_angles[nonzero_mask]
    left_ranges = left_ranges[nonzero_mask]
    x = left_ranges * np.cos(left_angles)
    y = left_ranges * np.sin(left_angles)


    if x.size > 0:
        scan_points = np.vstack((x, y))

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