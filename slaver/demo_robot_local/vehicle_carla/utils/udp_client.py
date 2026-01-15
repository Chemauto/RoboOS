"""UDP Client for CARLA Communication"""
import socket
import json
from typing import Dict, Optional


class UDPClient:
    """UDP client for sending control commands to CARLA"""

    def __init__(self, host: str = "192.168.1.1", port: int = 23456):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)

    def send_control(self, steer: float, throttle: float, brake: float) -> bool:
        """Send control command to CARLA

        Args:
            steer: Steering angle [-1.0, 1.0]
            throttle: Throttle [0.0, 1.0]
            brake: Brake [0.0, 1.0]

        Returns:
            Success status
        """
        try:
            control_msg = {
                "id": "control",
                "steer": float(steer),
                "throttle": float(throttle),
                "brake": float(brake)
            }
            data = json.dumps(control_msg).encode('utf-8')
            self.sock.sendto(data, (self.host, self.port))
            return True
        except Exception as e:
            print(f"[UDP] Send error: {e}")
            return False

    def close(self):
        """Close UDP socket"""
        self.sock.close()
