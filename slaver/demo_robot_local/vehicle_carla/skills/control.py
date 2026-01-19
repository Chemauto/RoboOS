"""Vehicle Control Skills"""
from ..utils.udp_client import UDPClient
from ..utils.logger import setup_logger
import socket
import json
import time
import math

logger = setup_logger("vehicle_control")


class VehicleController:
    """Vehicle control functionality"""

    def __init__(self, host: str = "127.0.0.1", port: int = 23456, sensor_port: int = 12345):
        self.udp_client = UDPClient(host, port)
        self.sensor_port = sensor_port
        self.sensor_sock = None

        # 尝试绑定传感器端口(用于反馈控制)
        try:
            self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sensor_sock.bind(("0.0.0.0", sensor_port))
            self.sensor_sock.settimeout(0.1)
            logger.info(f"VehicleController initialized: {host}:{port}, sensor:{sensor_port}")
        except Exception as e:
            logger.warning(f"VehicleController initialized without sensor binding: {e}")
            logger.info(f"VehicleController initialized: {host}:{port} (no sensor feedback)")

    def _get_sensor_data(self, data_type: str, timeout: float = 1.0):
        """Get sensor data of specific type"""
        if not self.sensor_sock:
            logger.warning("Sensor socket not available")
            return None

        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, _ = self.sensor_sock.recvfrom(4096)
                msg = json.loads(data.decode('utf-8'))
                if msg.get("id") == data_type:
                    return msg
            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"Sensor read error: {e}")
        return None

    def _get_heading(self) -> float:
        """Get current heading in degrees"""
        imu_data = self._get_sensor_data("imu", timeout=0.5)
        if imu_data:
            return imu_data.get("heading_deg", 0.0)
        return None

    def _get_position(self):
        """Get current position (x, y)"""
        gnss_data = self._get_sensor_data("gnss", timeout=0.5)
        if gnss_data:
            return (gnss_data.get("x", 0.0), gnss_data.get("y", 0.0))
        return None

    def set_vehicle_control(self, steer: float, throttle: float, brake: float) -> str:
        """Set vehicle control command

        Args:
            steer: Steering angle [-1.0, 1.0]
            throttle: Throttle [0.0, 1.0]
            brake: Brake [0.0, 1.0]

        Returns:
            Execution status
        """
        steer = max(-1.0, min(1.0, steer))
        throttle = max(0.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))

        success = self.udp_client.send_control(steer, throttle, brake)
        if success:
            logger.info(f"Control sent: steer={steer:.2f}, throttle={throttle:.2f}, brake={brake:.2f}")
            return f"Control command sent successfully: steer={steer:.2f}, throttle={throttle:.2f}, brake={brake:.2f}"
        else:
            logger.error("Failed to send control command")
            return "Failed to send control command"

    def emergency_brake(self) -> str:
        """Emergency brake

        Returns:
            Brake status
        """
        success = self.udp_client.send_control(0.0, 0.0, 1.0)
        if success:
            logger.warning("Emergency brake activated")
            return "Emergency brake activated successfully"
        else:
            logger.error("Failed to activate emergency brake")
            return "Failed to activate emergency brake"

    def stop_vehicle(self) -> str:
        """Stop vehicle

        Returns:
            Stop status
        """
        success = self.udp_client.send_control(0.0, 0.0, 1.0)
        if success:
            logger.info("Vehicle stopped")
            return "Vehicle stopped successfully"
        else:
            logger.error("Failed to stop vehicle")
            return "Failed to stop vehicle"

    def move_forward(self, speed: float) -> str:
        """Move forward to specified speed with feedback control

        Args:
            speed: Target speed in m/s

        Returns:
            Current speed status
        """
        throttle = min(speed / 10.0, 1.0)

        # 发送控制指令
        success = self.udp_client.send_control(0.0, throttle, 0.0)
        if success:
            logger.info(f"Moving forward: speed={speed:.2f} m/s, throttle={throttle:.2f}")
            return f"Moving forward at {speed:.2f} m/s (throttle={throttle:.2f})"
        else:
            logger.error("Failed to move forward")
            return "Failed to move forward"

    def move_forward_distance(self, distance: float, speed: float = 2.0) -> str:
        """Move forward for specified distance with feedback

        Args:
            distance: Target distance in meters
            speed: Speed in m/s (default 2.0)

        Returns:
            Execution status
        """
        throttle = min(speed / 10.0, 1.0)

        # 获取起始位置
        start_pos = self._get_position()
        if not start_pos:
            return "Failed to get starting position"

        logger.info(f"Moving {distance:.2f}m at {speed:.2f}m/s from {start_pos}")

        # 持续控制直到达到目标距离
        max_time = distance / speed * 3  # 最大时间=理论时间*3
        start_time = time.time()

        while time.time() - start_time < max_time:
            # 发送控制指令
            self.udp_client.send_control(0.0, throttle, 0.0)

            # 检查当前位置
            current_pos = self._get_position()
            if current_pos:
                traveled = math.sqrt((current_pos[0] - start_pos[0])**2 +
                                   (current_pos[1] - start_pos[1])**2)

                if traveled >= distance:
                    # 到达目标,停车
                    self.udp_client.send_control(0.0, 0.0, 1.0)
                    logger.info(f"Reached target: traveled {traveled:.2f}m")
                    return f"Successfully moved {traveled:.2f}m"

            time.sleep(0.05)  # 20Hz控制频率

        # 超时,停车
        self.udp_client.send_control(0.0, 0.0, 1.0)
        logger.warning("Move forward timeout")
        return "Move forward timeout"

    def turn_vehicle(self, angle: float) -> str:
        """Turn vehicle to specified angle with feedback control

        Args:
            angle: Turn angle in degrees (positive=right, negative=left)

        Returns:
            Current steering angle
        """
        steer = max(-1.0, min(1.0, angle / 45.0))

        # 发送控制指令
        success = self.udp_client.send_control(steer, 0.3, 0.0)
        if success:
            logger.info(f"Turning: angle={angle:.2f} degrees, steer={steer:.2f}")
            return f"Turning at {angle:.2f} degrees (steer={steer:.2f})"
        else:
            logger.error("Failed to turn vehicle")
            return "Failed to turn vehicle"

    def turn_vehicle_angle(self, target_angle: float, speed: float = 0.3) -> str:
        """Turn vehicle by specified angle with feedback

        Args:
            target_angle: Target turn angle in degrees (positive=right, negative=left)
            speed: Throttle during turn (default 0.3)

        Returns:
            Execution status
        """
        # 获取起始朝向
        start_heading = self._get_heading()
        if start_heading is None:
            return "Failed to get starting heading"

        target_heading = (start_heading + target_angle) % 360
        steer = 1.0 if target_angle > 0 else -1.0

        logger.info(f"Turning {target_angle:.2f}° from {start_heading:.2f}° to {target_heading:.2f}°")

        # 持续控制直到达到目标角度
        max_time = abs(target_angle) / 30.0 * 3  # 假设30°/秒转速,最大时间*3
        start_time = time.time()

        while time.time() - start_time < max_time:
            # 发送控制指令
            self.udp_client.send_control(steer, speed, 0.0)

            # 检查当前朝向
            current_heading = self._get_heading()
            if current_heading is not None:
                # 计算角度差(考虑360度循环)
                angle_diff = (target_heading - current_heading + 180) % 360 - 180

                if abs(angle_diff) < 5.0:  # 5度容差
                    # 到达目标,停车
                    self.udp_client.send_control(0.0, 0.0, 1.0)
                    logger.info(f"Reached target heading: {current_heading:.2f}°")
                    return f"Successfully turned to {current_heading:.2f}°"

            time.sleep(0.05)  # 20Hz控制频率

        # 超时,停车
        self.udp_client.send_control(0.0, 0.0, 1.0)
        logger.warning("Turn timeout")
        return "Turn timeout"
