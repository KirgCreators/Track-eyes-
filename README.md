---

# 🛠️ TrackEye-LiDAR

A **Final Year Project (2025)** by a Mechatronics student, integrating **ROS (Robot Operating System)** and **Python** for **rail surface scanning using a LIDAR sensor**.

---

## 📁 Project Structure

* **ROS Node**: `trackeye_rplidar2.py`
* **Launch File (Optional)**: `trackeye_lidar.launch`

---

## 🚀 Getting Started

### 1. **Start ROS Core**

```bash
roscore
```

### 2. **Connect to LIDAR**

Ensure your LIDAR is properly connected and recognized by the system. For example, using RPLIDAR:

```bash
roslaunch rplidar_ros rplidar.launch
```

> ⚠️ Adjust the launch file parameters to match your device port (e.g., `/dev/ttyUSB0`) and frame ID.

### 3. **Run the LIDAR Visualization Node**

```bash
rosrun trackeye_lidar trackeye_rplidar.py
```

> The Python script will display a live **scatter plot of the rail surface**, showing points detected on the left side of the sensor.

---

## 🐞 Troubleshooting

### ❗ Python Script Not Executable

If the Python file does not execute, make sure it has execute permissions:

```bash
chmod +x /home/<user>/catkin_ws/src/trackeye_lidar/scripts/trackeye_rplidar.py
```

Replace `<user>` with your actual username.

---

### ❗ Plot Shows Nothing

* Ensure the **angle mask matches your LIDAR orientation** in `trackeye_rplidar.py`.
  Example for left side of a 360° scan:

  ```python
  mask = (angles_rad >= 0) & (angles_rad <= np.pi)
  ```

* Make sure the LIDAR is publishing valid ranges:

```bash
rostopic echo /scan
```

---

## ⚙️ Auto Start Configuration (Optional)

This project can **auto-start visualization** on system boot.

### 🔧 `auto_start.launch` (Located in `launch/` folder)

```xml
<launch>
    <!-- Start LIDAR driver -->
    <include file="$(find rplidar_ros)/launch/rplidar.launch" />

    <!-- Start visualization node -->
    <node pkg="trackeye_lidar" type="trackeye_rplidar.py" name="trackeye_lidar_visual" output="screen" />
</launch>
```

> Make sure device ports are correct.

---

### 🧠 Autostart Script

Create a startup entry using your OS’s Startup Applications:

* **Name**: `TrackEye-LiDAR Autostart`
* **Command**:

```bash
gnome-terminal -- bash -c "/home/$USER/start_trackeye_lidar.sh; exec bash"
```

---

### 📜 `start_trackeye_lidar.sh`

Place this script in your home directory (`~/start_trackeye_lidar.sh`) and make it executable:

```bash
#!/bin/bash

# Load ROS environment
source /opt/ros/noetic/setup.bash
source /home/$USER/catkin_ws/devel/setup.bash

# Launch LIDAR visualization
roslaunch trackeye_lidar auto_start.launch
```

Make it executable:

```bash
chmod +x ~/start_trackeye_lidar.sh
```

---

## 📌 Requirements

* ROS Noetic
* LIDAR sensor (e.g., RPLIDAR)
* Python 3
* `numpy`, `matplotlib`
* `rplidar_ros` ROS package

---
