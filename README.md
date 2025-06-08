# Xsens IMU ROS 2 Publisher
This project provides a ROS 2 Python node that interfaces with an **Xsens MTi-300** device via the Xsens Device API (XDA). It publishes real-time IMU data (orientation, acceleration, gyroscope) to `/imu/data`, and prints additional diagnostic data such as magnetometer, GPS, altitude, and ENU velocity to the terminal.

---

## 📦 Features

- Uses `xsensdeviceapi` to access MTi-300 sensor data.
- Publishes to standard `sensor_msgs/msg/Imu` topic.
- Displays all available sensor readings in terminal.
- Works with ROS 2 and Python 3.

---

## 🚀 Requirements

- Xsens MTi-300 device (connected via USB).
- ROS 2 Humble (or later).
- Python 3.8+
- `xsensdeviceapi` Python bindings installed.

Install dependencies:

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs
```

---

## 🛠️ Build and Run

### 1. Clone or copy into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/lonelyfluency/xsens_imu_publisher.git xsens_imu_publisher
```

### 2. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select xsens_imu_publisher
source install/setup.bash
```

### 3. Run the node

```bash
ros2 run xsens_imu_publisher xsens_node
```

---

## 📤 Published Topics

| Topic       | Message Type         | Description                        |
|-------------|----------------------|------------------------------------|
| `/imu/data` | `sensor_msgs/Imu`    | Orientation, Angular Vel, Accel    |

---

## 🖨️ Terminal Output

The following extra sensor data will be printed:

- Orientation (quaternion and Euler)
- Linear acceleration (x, y, z)
- Angular velocity (x, y, z)
- Magnetic field

---
