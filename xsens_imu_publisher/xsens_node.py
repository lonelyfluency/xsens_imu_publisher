#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time

import xsensdeviceapi as xda
from threading import Lock
import math


class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size=10):
        super().__init__()
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = []
        self.m_lock = Lock()

    def packetAvailable(self):
        with self.m_lock:
            return len(self.m_packetBuffer) > 0

    def getNextPacket(self):
        with self.m_lock:
            assert len(self.m_packetBuffer) > 0
            return self.m_packetBuffer.pop(0)

    def onLiveDataAvailable(self, dev, packet):
        with self.m_lock:
            if len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
                self.m_packetBuffer.pop(0)
            self.m_packetBuffer.append(xda.XsDataPacket(packet))


class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('xsens_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

        self.control = xda.XsControl_construct()
        assert self.control is not None

        # Scan for MTi device
        port_info_array = xda.XsScanner_scanPorts()
        self.mtPort = None
        for i in range(port_info_array.size()):
            pi = port_info_array[i]
            if pi.deviceId().isMti() or pi.deviceId().isMtig():
                self.mtPort = pi
                break

        if self.mtPort is None or self.mtPort.empty():
            self.get_logger().error("No MTi device found.")
            raise RuntimeError("No MTi device found.")

        if not self.control.openPort(self.mtPort.portName(), self.mtPort.baudrate()):
            raise RuntimeError("Could not open port.")

        self.device = self.control.device(self.mtPort.deviceId())
        assert self.device is not None

        self.callback = XdaCallback()
        self.device.addCallbackHandler(self.callback)

        if not self.device.gotoConfig():
            raise RuntimeError("Could not enter config mode.")

        # Request all useful outputs
        configArray = xda.XsOutputConfigurationArray()
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, 100))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ, 100))

        if not self.device.setOutputConfiguration(configArray):
            raise RuntimeError("Failed to set output configuration.")

        if not self.device.gotoMeasurement():
            raise RuntimeError("Could not enter measurement mode.")

        self.get_logger().info("Xsens MTi initialized and publishing all sensor data.")

    def publish_imu_data(self):
        if self.callback.packetAvailable():
            packet = self.callback.getNextPacket()
            imu_msg = Imu()
            now = self.get_clock().now().to_msg()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'imu_link'

            display_info = []

            # Orientation
            if packet.containsOrientation():
                q = packet.orientationQuaternion()
                imu_msg.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
                euler = packet.orientationEuler()
                display_info.append(f"Orientation q: [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]")
                display_info.append(f"Euler: Roll={euler.x():.3f}°, Pitch={euler.y():.3f}°, Yaw={euler.z():.3f}°")

            # Acceleration and Gyroscope
            if packet.containsCalibratedData():
                acc = packet.calibratedAcceleration()
                gyr = packet.calibratedGyroscopeData()
                imu_msg.linear_acceleration.x = acc[0]
                imu_msg.linear_acceleration.y = acc[1]
                imu_msg.linear_acceleration.z = acc[2]
                imu_msg.angular_velocity.x = gyr[0]
                imu_msg.angular_velocity.y = gyr[1]
                imu_msg.angular_velocity.z = gyr[2]
                display_info.append(f"Accel [m/s²]: x={acc[0]:.3f}, y={acc[1]:.3f}, z={acc[2]:.3f}")
                display_info.append(f"Gyro [rad/s]: x={gyr[0]:.3f}, y={gyr[1]:.3f}, z={gyr[2]:.3f}")

            # Magnetometer
            if packet.containsRawMagneticField():
                mag = packet.calibratedMagneticField()
                display_info.append(f"Mag [G]: x={mag[0]:.3f}, y={mag[1]:.3f}, z={mag[2]:.3f}")

            # GNSS - Latitude/Longitude
            if packet.containsLatitudeLongitude():
                latlon = packet.latitudeLongitude()
                display_info.append(f"Lat/Lon: {latlon[0]:.6f}, {latlon[1]:.6f}")

            # Altitude
            if packet.containsAltitude():
                alt = packet.altitude()
                display_info.append(f"Altitude: {alt:.3f} m")

            # Velocity (ENU)
            if packet.containsVelocity():
                vel = packet.velocity(xda.XDI_CoordSysEnu)
                display_info.append(f"Velocity [m/s]: E={vel[0]:.3f}, N={vel[1]:.3f}, U={vel[2]:.3f}")

            # Publish to ROS 2
            self.publisher_.publish(imu_msg)

            # Print all values
            print("\n".join(display_info), end="\n\n")

    def destroy_node(self):
        self.get_logger().info("Shutting down Xsens IMU node...")
        try:
            self.device.removeCallbackHandler(self.callback)
            self.control.closePort(self.mtPort.portName())
            self.control.close()
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuPublisherNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info("Keyboard interrupt received.")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
