#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time
import math

scan_points = None
last_scan_time = 0
scan_buffer = []
plot_active = True

# Third rail inspection parameters
NOMINAL_DISTANCE = 124.0  # mm - your fixed reference distance
THICKNESS_TOLERANCE = 6.0  # mm - allowable deviation from nominal
CRACK_THRESHOLD = 10.0     # mm - sudden jump indicating crack
MIN_CRACK_WIDTH = 2.0      # mm - minimum width to be considered a crack
PROBLEM_AREA_SIZE = 18     # mm - area to analyze for problems

# Problem tracking
problem_detected = False
current_problem_severity = 0.0
problem_locations = []
inspection_data = []

def detect_problems(x_points, y_points, distances):
    """
    Detect third rail problems based on distance deviations
    Returns: problem_type, severity, location_x, problem_details
    """
    global NOMINAL_DISTANCE, THICKNESS_TOLERANCE, CRACK_THRESHOLD
    
    if len(distances) < 5:
        return "insufficient_data", 0, 0, {}
    
    problems = []
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    
    # Problem 1: Overall thinning/thickening
    overall_deviation = abs(mean_distance - NOMINAL_DISTANCE)
    if overall_deviation > THICKNESS_TOLERANCE:
        problem_type = "thinning" if mean_distance > NOMINAL_DISTANCE else "thickening"
        severity = min(overall_deviation / THICKNESS_TOLERANCE, 3.0)  # Scale severity
        problems.append((problem_type, severity, np.mean(x_points), 
                        f"Mean distance: {mean_distance:.1f}mm vs nominal {NOMINAL_DISTANCE}mm"))
    
    # Problem 2: Localized cracks/thinning
    if len(x_points) > 10:
        # Sort points by X position for continuity analysis
        sorted_indices = np.argsort(x_points)
        x_sorted = x_points[sorted_indices]
        y_sorted = y_points[sorted_indices]
        dist_sorted = distances[sorted_indices]
        
        # Detect sudden jumps (cracks)
        for i in range(1, len(x_sorted)-1):
            if abs(x_sorted[i] - x_sorted[i-1]) < 5:  # Points are close in X
                dist_diff = abs(dist_sorted[i] - dist_sorted[i-1])
                if dist_diff > CRACK_THRESHOLD:
                    crack_width = abs(x_sorted[i] - x_sorted[i-1])
                    if crack_width >= MIN_CRACK_WIDTH:
                        severity = min(dist_diff / CRACK_THRESHOLD, 3.0)
                        problems.append(("crack", severity, (x_sorted[i] + x_sorted[i-1])/2,
                                       f"Crack detected: {dist_diff:.1f}mm jump over {crack_width:.1f}mm"))
        
        # Detect continuous thinning areas
        window_size = min(10, len(x_sorted)//3)
        for i in range(len(x_sorted) - window_size + 1):
            window_distances = dist_sorted[i:i+window_size]
            window_mean = np.mean(window_distances)
            if window_mean > NOMINAL_DISTANCE + THICKNESS_TOLERANCE:
                # Check if this is a sustained deviation
                deviations = window_distances - NOMINAL_DISTANCE
                if np.all(deviations > THICKNESS_TOLERANCE/2):
                    severity = min((window_mean - NOMINAL_DISTANCE) / THICKNESS_TOLERANCE, 2.0)
                    problems.append(("local_thinning", severity, np.mean(x_sorted[i:i+window_size]),
                                   f"Local thinning: {window_mean:.1f}mm over {window_size} points"))
    
    # Return the most severe problem
    if problems:
        problems.sort(key=lambda x: x[1], reverse=True)  # Sort by severity
        return problems[0]
    
    return "normal", 0, 0, {}

def callback(scan):
    global scan_points, last_scan_time, scan_buffer, problem_detected, current_problem_severity, problem_locations
    
    try:
        # Original angles from LIDAR
        angles_rad = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges_m = np.array(scan.ranges)
        
        # Better handling of invalid measurements
        valid_mask = np.isfinite(ranges_m) & (ranges_m > 0) & (ranges_m < scan.range_max)
        ranges_m = ranges_m[valid_mask]
        angles_rad = angles_rad[valid_mask]

        # Convert to millimeters for better precision
        ranges_mm = ranges_m * 1000.0

        # Target range: 100-150 mm (focused around nominal 130mm)
        min_mm = 100
        max_mm = 150
        range_mask = (ranges_mm >= min_mm) & (ranges_mm <= max_mm)
        
        if np.sum(range_mask) < 3:  # Too few points in target range
            return
            
        ranges_mm = ranges_mm[range_mask]
        angles_rad = angles_rad[range_mask]

        # Manual statistical filtering
        if len(ranges_mm) > 5:
            mean_dist = np.mean(ranges_mm)
            std_dist = np.std(ranges_mm)
            # Keep points within 2 standard deviations
            statistical_mask = (ranges_mm >= mean_dist - 2*std_dist) & (ranges_mm <= mean_dist + 2*std_dist)
            ranges_mm = ranges_mm[statistical_mask]
            angles_rad = angles_rad[statistical_mask]

        # Convert to Cartesian (in mm)
        x = ranges_mm * np.cos(angles_rad)
        y = ranges_mm * np.sin(angles_rad)

        # Rotate 90° clockwise
        theta = -np.pi / 2
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        if x_rot.size > 0:
            current_points = np.column_stack((x_rot, y_rot, ranges_mm))
            
            # Simple temporal smoothing
            scan_buffer.append(current_points)
            
            # Keep only last 5 scans for smoothing
            if len(scan_buffer) > 5:
                scan_buffer.pop(0)
                
            # Combine points from multiple scans
            if len(scan_buffer) > 0:
                all_points = np.vstack(scan_buffer)
                
                # Simple distance-based filtering to remove isolated points
                if all_points.shape[0] > 10:
                    from collections import defaultdict
                    point_groups = defaultdict(list)
                    
                    # Group points by rounded coordinates (simple binning)
                    for i, point in enumerate(all_points):
                        key = (round(point[0]/5)*5, round(point[1]/5)*5)  # 5mm bins
                        point_groups[key].append(i)
                    
                    # Keep only bins with multiple points
                    keep_indices = []
                    for key, indices in point_groups.items():
                        if len(indices) >= 2:  # Keep bins with at least 2 points
                            keep_indices.extend(indices)
                    
                    if keep_indices:
                        all_points = all_points[keep_indices]
                
                scan_points = all_points.T  # Convert back to column format
                
                # Detect problems with current scan data
                if scan_points is not None and scan_points.shape[1] > 5:
                    problem_type, severity, location, details = detect_problems(
                        scan_points[0], scan_points[1], scan_points[2]
                    )
                    
                    problem_detected = (severity > 0.5)  # Only alert for significant issues
                    current_problem_severity = severity
                    
                    if problem_detected and severity > 1.0:  # Only log serious problems
                        timestamp = time.time()
                        problem_locations.append((timestamp, location, problem_type, severity, details))
                        
                        # Log the problem
                        rospy.logwarn(f"🚨 THIRD RAIL PROBLEM DETECTED: {problem_type.upper()}")
                        rospy.logwarn(f"   Severity: {severity:.1f}, Location: {location:.1f}mm")
                        rospy.logwarn(f"   Details: {details}")
                        rospy.logwarn(f"   Timestamp: {timestamp}")
                        
                        # Store inspection data
                        inspection_data.append({
                            'timestamp': timestamp,
                            'problem_type': problem_type,
                            'severity': severity,
                            'location': location,
                            'mean_distance': np.mean(scan_points[2]),
                            'details': details
                        })
            
            last_scan_time = time.time()
            
    except Exception as e:
        rospy.logwarn(f"Error processing scan: {e}")

def update_plot():
    global scan_points, last_scan_time, plot_active, problem_detected, current_problem_severity
    
    try:
        plt.clf()

        # Create main plot
        plt.subplot(2, 1, 1)  # 2 rows, 1 column, first subplot
        
        if scan_points is not None and time.time() - last_scan_time < 2.0:
            x_vals = scan_points[0]
            y_vals = scan_points[1]
            distances = scan_points[2]

            # Color points based on problem detection
            point_colors = distances
            if problem_detected:
                # Highlight problem areas in red
                cmap = 'RdYlGn_r'  # Red for problems, green for normal
            else:
                cmap = 'viridis'
            
            scatter = plt.scatter(x_vals, y_vals, c=point_colors, cmap=cmap, 
                                s=30, alpha=0.7, vmin=100, vmax=150)
            
            # Add trend line if enough points
            if len(x_vals) > 2:
                sort_idx = np.argsort(x_vals)
                x_sorted = x_vals[sort_idx]
                y_sorted = y_vals[sort_idx]
                
                if len(x_sorted) > 5:
                    window_size = min(5, len(x_sorted) // 3)
                    if window_size > 1:
                        y_smooth = np.convolve(y_sorted, np.ones(window_size)/window_size, mode='valid')
                        x_smooth = x_sorted[window_size-1:]
                        plt.plot(x_smooth, y_smooth, 'r-', alpha=0.5, linewidth=2, label='Surface profile')
            
            # Add nominal distance reference line
            plt.axhline(y=NOMINAL_DISTANCE, color='blue', linestyle='-', alpha=0.7, 
                       label=f'Nominal: {NOMINAL_DISTANCE}mm')
            plt.axhline(y=NOMINAL_DISTANCE + THICKNESS_TOLERANCE, color='orange', 
                       linestyle='--', alpha=0.5, label='Tolerance limit')
            plt.axhline(y=NOMINAL_DISTANCE - THICKNESS_TOLERANCE, color='orange', 
                       linestyle='--', alpha=0.5)

        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.title("Third Rail Surface Inspection (100-150 mm range)")
        plt.xlim(-50, 50)
        plt.ylim(80, 180)
        plt.grid(True, linestyle='--', alpha=0.3)
        plt.legend()
        
        if scan_points is not None and time.time() - last_scan_time < 2.0:
            plt.colorbar(scatter, label='Distance (mm)')

        # Add status subplot
        plt.subplot(2, 1, 2)
        plt.axis('off')
        
        # Display status information
        status_text = []
        if scan_points is not None and time.time() - last_scan_time < 2.0:
            mean_dist = np.mean(scan_points[2])
            std_dist = np.std(scan_points[2])
            points_count = len(scan_points[0])
            
            status_text.append(f"Current Mean Distance: {mean_dist:.1f} mm")
            status_text.append(f"Nominal Distance: {NOMINAL_DISTANCE} mm")
            status_text.append(f"Deviation: {abs(mean_dist - NOMINAL_DISTANCE):.1f} mm")
            status_text.append(f"Points Analyzed: {points_count}")
            status_text.append(f"Standard Deviation: {std_dist:.1f} mm")
            
            if problem_detected:
                status_text.append(f"🚨 PROBLEM DETECTED - Severity: {current_problem_severity:.1f}")
                status_text.append(f"Action: INSPECT RAIL SURFACE")
            else:
                status_text.append("✅ STATUS: NORMAL")
                
            # Add recent problems
            if problem_locations:
                status_text.append("\nRecent Problems:")
                for i, (ts, loc, ptype, sev, det) in enumerate(problem_locations[-3:]):  # Last 3 problems
                    status_text.append(f"  {ptype}: sev={sev:.1f} @ {loc:.1f}mm")
        else:
            status_text.append("Waiting for scan data...")
        
        plt.text(0.02, 0.95, "\n".join(status_text), transform=plt.gca().transAxes,
                verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow"),
                fontfamily='monospace', fontsize=9)

        plt.tight_layout()
        plt.pause(0.05)
        
    except Exception as e:
        rospy.logwarn(f"Plot update error: {e}")
        plot_active = False

def close_plot():
    """Properly close the plot"""
    global plot_active
    plot_active = False
    plt.close('all')
    rospy.loginfo("Plot closed")

if __name__ == '__main__':
    rospy.init_node('third_rail_inspection')
    rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)

    # Set up plot with proper close handling
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    
    def on_close(event):
        global plot_active
        plot_active = False
        rospy.loginfo("Plot window closed by user")
    
    fig.canvas.mpl_connect('close_event', on_close)
    
    print("=" * 60)
    print("THIRD RAIL INSPECTION SYSTEM STARTED")
    print("=" * 60)
    print(f"Nominal Distance: {NOMINAL_DISTANCE} mm")
    print(f"Tolerance: ±{THICKNESS_TOLERANCE} mm")
    print(f"Crack Threshold: {CRACK_THRESHOLD} mm")
    print("Close the plot window or press Ctrl+C to exit")
    print("=" * 60)

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown() and plot_active:
            try:
                update_plot()
                rate.sleep()
            except Exception as e:
                if "application has been destroyed" in str(e):
                    rospy.loginfo("Plot window closed")
                    break
                else:
                    rospy.logerr(f"Error in main loop: {e}")
                    rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        # Print inspection summary
        if inspection_data:
            print("\n" + "=" * 60)
            print("INSPECTION SUMMARY")
            print("=" * 60)
            for data in inspection_data:
                print(f"Time: {time.ctime(data['timestamp'])}")
                print(f"Problem: {data['problem_type']} (Severity: {data['severity']:.1f})")
                print(f"Location: {data['location']:.1f}mm, Mean Distance: {data['mean_distance']:.1f}mm")
                print(f"Details: {data['details']}")
                print("-" * 40)
        
        close_plot()
        rospy.loginfo("Third rail inspection system shutdown complete")