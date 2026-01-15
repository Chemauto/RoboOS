"""Sensor Data Skills"""
import socket
import json
from typing import Dict, Optional
from ..utils.logger import setup_logger

logger = setup_logger("vehicle_sensor")


class SensorReader:
    """Sensor data reading functionality"""

    def __init__(self, host: str = "192.168.1.101", port: int = 12345):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)
        self.latest_gnss = None
        self.latest_imu = None
        logger.info(f"SensorReader initialized: {host}:{port}")

    def _receive_sensor_data(self) -> Optional[Dict]:
        """Receive sensor data from UDP"""
        try:
            data, _ = self.sock.recvfrom(4096)
            msg = json.loads(data.decode('utf-8'))
            return msg
        except socket.timeout:
            logger.warning("Sensor data receive timeout")
            return None
        except Exception as e:
            logger.error(f"Error receiving sensor data: {e}")
            return None

    def get_gnss_data(self) -> str:
        """Get GPS position data

        Returns:
            x, y, z coordinates
        """
        msg = self._receive_sensor_data()
        if msg and msg.get("id") == "gnss":
            self.latest_gnss = msg
            x, y, z = msg.get("x", 0), msg.get("y", 0), msg.get("z", 0)
            logger.info(f"GNSS data: x={x:.6f}, y={y:.6f}, z={z:.6f}")
            return f"GNSS position: x={x:.6f}, y={y:.6f}, z={z:.6f}"
        elif self.latest_gnss:
            x, y, z = self.latest_gnss.get("x", 0), self.latest_gnss.get("y", 0), self.latest_gnss.get("z", 0)
            return f"Latest GNSS position: x={x:.6f}, y={y:.6f}, z={z:.6f}"
        else:
            logger.warning("No GNSS data available")
            return "No GNSS data available"

    def get_imu_data(self) -> str:
        """Get IMU data

        Returns:
            Acceleration, angular velocity, heading
        """
        msg = self._receive_sensor_data()
        if msg and msg.get("id") == "imu":
            self.latest_imu = msg
            accel = msg.get("accelerometer", {})
            gyro = msg.get("gyroscope", {})
            heading = msg.get("heading_deg", 0)
            logger.info(f"IMU data: heading={heading:.2f} deg")
            return (f"IMU data - Accel: ({accel.get('x', 0):.2f}, {accel.get('y', 0):.2f}, {accel.get('z', 0):.2f}), "
                    f"Gyro: ({gyro.get('x', 0):.2f}, {gyro.get('y', 0):.2f}, {gyro.get('z', 0):.2f}), "
                    f"Heading: {heading:.2f} deg")
        elif self.latest_imu:
            accel = self.latest_imu.get("accelerometer", {})
            gyro = self.latest_imu.get("gyroscope", {})
            heading = self.latest_imu.get("heading_deg", 0)
            return (f"Latest IMU data - Accel: ({accel.get('x', 0):.2f}, {accel.get('y', 0):.2f}, {accel.get('z', 0):.2f}), "
                    f"Gyro: ({gyro.get('x', 0):.2f}, {gyro.get('y', 0):.2f}, {gyro.get('z', 0):.2f}), "
                    f"Heading: {heading:.2f} deg")
        else:
            logger.warning("No IMU data available")
            return "No IMU data available"

    def get_vehicle_status(self) -> str:
        """Get vehicle status

        Returns:
            Position, speed, heading
        """
        status_parts = []

        # Get GNSS data
        if self.latest_gnss:
            x, y, z = self.latest_gnss.get("x", 0), self.latest_gnss.get("y", 0), self.latest_gnss.get("z", 0)
            status_parts.append(f"Position: ({x:.6f}, {y:.6f}, {z:.6f})")
        else:
            status_parts.append("Position: N/A")

        # Get IMU data
        if self.latest_imu:
            heading = self.latest_imu.get("heading_deg", 0)
            status_parts.append(f"Heading: {heading:.2f} deg")
        else:
            status_parts.append("Heading: N/A")

        result = "Vehicle Status - " + ", ".join(status_parts)
        logger.info(result)
        return result
