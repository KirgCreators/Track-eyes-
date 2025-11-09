#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

def callback(scan):
    
    target_angle = np.deg2rad(270)  # RPlidar left side
    angle_tolerance = np.deg2rad(1)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    mask = np.abs(angles - target_angle) <= angle_tolerance
    values = np.array(scan.ranges)[mask]
    values = values[np.isfinite(values)]
    if values.size == 0:
        return
    distance = np.median(values)

    # publish
    pub.publish(distance)
    rospy.loginfo("Rail distance: {:.3f} m".format(distance))

rospy.init_node('trackeye_rplidar')
pub = rospy.Publisher('/rail_height', Float32, queue_size=10)
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
