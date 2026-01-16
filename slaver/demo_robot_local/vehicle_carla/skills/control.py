"""Vehicle Control Skills"""
from ..utils.udp_client import UDPClient
from ..utils.logger import setup_logger

logger = setup_logger("vehicle_control")


class VehicleController:
    """Vehicle control functionality"""

    def __init__(self, host: str = "127.0.0.1", port: int = 23456):
        self.udp_client = UDPClient(host, port)
        logger.info(f"VehicleController initialized: {host}:{port}")

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
        """Move forward to specified speed

        Args:
            speed: Target speed in m/s

        Returns:
            Current speed status
        """
        throttle = min(speed / 10.0, 1.0)
        success = self.udp_client.send_control(0.0, throttle, 0.0)
        if success:
            logger.info(f"Moving forward: speed={speed:.2f} m/s, throttle={throttle:.2f}")
            return f"Moving forward at {speed:.2f} m/s (throttle={throttle:.2f})"
        else:
            logger.error("Failed to move forward")
            return "Failed to move forward"

    def turn_vehicle(self, angle: float) -> str:
        """Turn vehicle to specified angle

        Args:
            angle: Turn angle in degrees (positive=right, negative=left)

        Returns:
            Current steering angle
        """
        steer = max(-1.0, min(1.0, angle / 45.0))
        success = self.udp_client.send_control(steer, 0.3, 0.0)
        if success:
            logger.info(f"Turning: angle={angle:.2f} degrees, steer={steer:.2f}")
            return f"Turning at {angle:.2f} degrees (steer={steer:.2f})"
        else:
            logger.error("Failed to turn vehicle")
            return "Failed to turn vehicle"
