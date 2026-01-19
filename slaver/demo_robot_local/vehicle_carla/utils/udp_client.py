"""UDP Client for CARLA Communication"""
import socket
import json
from typing import Dict, Optional


class UDPClient:
    """UDP client for sending control commands to CARLA"""

    def __init__(self, host: str = "127.0.0.1", port: int = 23456):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)

    def send_control(self, steer: float, throttle: float, brake: float, reverse: bool = False) -> bool:
        """Send control command to CARLA

        Args:
            steer: Steering angle [-1.0, 1.0]
            throttle: Throttle [0.0, 1.0]
            brake: Brake [0.0, 1.0]
            reverse: Reverse gear (default False)

        Returns:
            Success status
        """
        try:
            control_msg = {
                "id": "control",
                "steer": float(steer),
                "throttle": float(throttle),
                "brake": float(brake),
                "reverse": bool(reverse)
            }
            data = json.dumps(control_msg).encode('utf-8')
            bytes_sent = self.sock.sendto(data, (self.host, self.port))

            # 写入文件日志
            with open("/tmp/udp_send.log", "a") as f:
                f.write(f"[UDP] Sent {bytes_sent} bytes to {self.host}:{self.port}: {control_msg}\n")
                f.flush()

            print(f"[UDP] Sent {bytes_sent} bytes to {self.host}:{self.port}: {control_msg}", flush=True)
            return True
        except Exception as e:
            with open("/tmp/udp_send.log", "a") as f:
                f.write(f"[UDP] Send error: {e}\n")
                f.flush()
            print(f"[UDP] Send error: {e}", flush=True)
            return False

    def close(self):
        """Close UDP socket"""
        self.sock.close()
