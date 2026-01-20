"""Sensor Data Skills"""
import socket
import json
from typing import Dict, Optional
from ..utils.logger import setup_logger

logger = setup_logger("vehicle_sensor")


class SensorReader:
    """Sensor data reading functionality"""

    def __init__(self, host: str = "127.0.0.1", port: int = 12347):
        """
        Initialize sensor reader

        Args:
            host: Host to receive data from (default: 127.0.0.1 for local CARLA)
            port: Port to listen on (default: 12347, matching simple_vehicle_control.py)

        Note: 修改历史
            - 旧配置: host="192.168.1.101", port=12345 (错误配置)
            - 新配置: host="127.0.0.1", port=12347 (与 simple_vehicle_control.py 匹配)
        """
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)

        # Bind to port to receive sensor data from CARLA
        try:
            self.sock.bind(("0.0.0.0", port))
            logger.info(f"SensorReader initialized and bound to port {port}")
        except OSError as e:
            logger.warning(f"Failed to bind to port {port}: {e}. Will try to receive without binding.")
            logger.info(f"SensorReader initialized: {host}:{port} (no bind)")

        self.latest_gnss = None
        self.latest_imu = None

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
        # Try to receive fresh sensor data (up to 4 packets to get both GNSS and IMU)
        for _ in range(4):
            msg = self._receive_sensor_data()
            if msg:
                if msg.get("id") == "gnss":
                    self.latest_gnss = msg
                elif msg.get("id") == "imu":
                    self.latest_imu = msg
            # Stop if we have both
            if self.latest_gnss and self.latest_imu:
                break

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

    def get_raw_sensor_data(self, data_type: str, timeout: float = 1.0) -> str:
        """Get raw sensor data in JSON format

        Args:
            data_type: "gnss" or "imu"
            timeout: Timeout in seconds (default 1.0)

        Returns:
            Raw sensor data as JSON string
        """
        import json

        # Set temporary timeout
        original_timeout = self.sock.gettimeout()
        self.sock.settimeout(timeout)

        try:
            data = self._receive_sensor_data()
            if data and data.get("id") == data_type:
                logger.info(f"Raw {data_type} data retrieved")
                return json.dumps(data, indent=2)
            else:
                logger.warning(f"Failed to get {data_type} data")
                return f"Failed to get {data_type} data"
        finally:
            # Restore original timeout
            self.sock.settimeout(original_timeout)

    def close_connection(self) -> str:
        """Close sensor socket connection

        Returns:
            Close status
        """
        try:
            if self.sock:
                self.sock.close()
                logger.info("Sensor socket closed")
                return "Sensor connection closed successfully"
        except Exception as e:
            logger.error(f"Error closing sensor connection: {e}")
            return f"Error closing sensor connection: {e}"
