#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import matplotlib
matplotlib.use('Agg')  # no GUI needed
import matplotlib.pyplot as plt



plt.ion()  # interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')
ax.set_ylim(0, 2)  # adjust based on rail distance range in meters
ax.set_xlim(0, 360)  # lidar angles
ax.set_xlabel("Angle (deg)")
ax.set_ylabel("Distance (m)")

def callback(scan):
    global line
    angles_deg = np.linspace(0, 360, len(scan.ranges))
    ranges = np.array(scan.ranges)
    ranges[~np.isfinite(ranges)] = 0

    # mask only one side
    mask = (angles_deg >= 90) & (angles_deg <= 180)

    line.set_xdata(angles_deg[mask])
    line.set_ydata(ranges[mask])

    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.001)

    # also publish median distance
    target_values = ranges[mask]
    if target_values.size > 0:
        pub.publish(np.median(target_values))
        rospy.loginfo("distance: {:.3f} m".format(np.median(target_values)))

rospy.init_node('trackeye_rplidar')
pub = rospy.Publisher('/rail_height', Float32, queue_size=10)
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
