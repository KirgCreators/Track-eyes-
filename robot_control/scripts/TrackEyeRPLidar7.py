#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt

# ------------------------------
# CONFIGURATION
# ------------------------------
MIN_RANGE = 0.120   # 120 mm
MAX_RANGE = 0.160   # 160 mm
CRACK_THRESHOLD = 0.003  # 3 mm deviation considered a crack
MEDIAN_KERNEL = 5
MOVING_AVG_WINDOW = 5

# Buffer for averaging multiple scans
SCAN_AVG_COUNT = 5
scan_buffer = []

scan_angles = None  # Will store lidars angle array

# ------------------------------
# LIDAR CALLBACK
# ------------------------------
def callback(scan: LaserScan):
    global scan_buffer, scan_angles

    ranges = np.array(scan.ranges)

    # Create angles (in radians)
    if scan_angles is None:
        scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    # Replace inf/nan with max lidar range
    ranges = np.nan_to_num(ranges, nan=scan.range_max, posinf=scan.range_max)

    # --------------------------
    # FILTER: median filter (spike removal)
    # --------------------------
    ranges = medfilt(ranges, kernel_size=MEDIAN_KERNEL)

    # --------------------------
    # FILTER: moving-average smoothing
    # --------------------------
    kernel = np.ones(MOVING_AVG_WINDOW) / MOVING_AVG_WINDOW
    ranges = np.convolve(ranges, kernel, mode="same")

    # --------------------------
    # LIMIT RANGE (zoom 120–160 mm only)
    # --------------------------
    mask = (ranges >= MIN_RANGE) & (ranges <= MAX_RANGE)
    angles_zoom = scan_angles[mask]
    ranges_zoom = ranges[mask]

    if len(ranges_zoom) == 0:
        return

    # Store in buffer
    scan_buffer.append(ranges_zoom)

    # If enough scans collected → average & plot
    if len(scan_buffer) >= SCAN_AVG_COUNT:
        avg_ranges = np.mean(scan_buffer, axis=0)
        scan_buffer = []  # reset buffer

        plot_scan(angles_zoom, avg_ranges)


# ------------------------------
# PLOTTING FUNCTION
# ------------------------------
def plot_scan(angles, ranges):
    plt.clf()  # clear frame

    # Convert radians to degrees for readability
    angles_deg = np.degrees(angles)

    # --------------------------
    # DETECT CRACKS
    # baseline: mean surface level
    # --------------------------
    baseline = np.mean(ranges)
    deviation = np.abs(ranges - baseline)
    cracks = deviation > CRACK_THRESHOLD

    # --------------------------
    # PLOT
    # --------------------------
    plt.plot(angles_deg, ranges, linewidth=1.0)

    # Mark cracks in red
    plt.scatter(angles_deg[cracks], ranges[cracks], c='red')

    # Zoom in y-axis tightly
    plt.ylim(MIN_RANGE, MAX_RANGE)

    # Gridlines for clarity
    plt.grid(True, linestyle="--", linewidth=0.5)

    plt.xlabel("Angle (degrees)")
    plt.ylabel("Distance (m)")
    plt.title("RPLIDAR Third Rail Surface Scan (120–160 mm zone)")

    plt.pause(0.01)  # non-blocking plot


# ------------------------------
# MAIN
# ------------------------------
def main():
    rospy.init_node("third_rail_analysis", anonymous=True)

    plt.ion()  # interactive mode
    plt.figure(figsize=(8, 4))

    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.loginfo("Third rail scanning node started.")

    rospy.spin()


if __name__ == "__main__":
    main()
