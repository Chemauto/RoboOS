#!/usr/bin/env python3
"""Check if CARLA vehicle is stopped"""
import sys
sys.path.insert(0, '/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla')

from utils.udp_client import UDPClient

def check_and_stop_vehicle():
    """Send stop command and verify"""
    print("=== Checking Vehicle Status ===\n")

    # Initialize UDP client
    print("1. Connecting to CARLA vehicle...")
    try:
        client = UDPClient(host="127.0.0.1", port=23456)
        print("   ✓ Connected to 127.0.0.1:23456\n")
    except Exception as e:
        print(f"   ✗ Failed to connect: {e}")
        return

    # Send stop command
    print("2. Sending STOP command (brake=1.0)...")
    success = client.send_control(steer=0.0, throttle=0.0, brake=1.0)

    if success:
        print("   ✓ Stop command sent successfully")
        print("\n=== Result ===")
        print("Vehicle should now be STOPPED (brake applied)")
        print("\nVerify in CARLA simulator:")
        print("- Vehicle should not be moving")
        print("- Brake should be engaged")
    else:
        print("   ✗ Failed to send stop command")
        print("\nPossible issues:")
        print("- CARLA simulation not running")
        print("- Network connection problem")
        print("- UDP port 23456 not listening")

    client.close()

if __name__ == "__main__":
    try:
        check_and_stop_vehicle()
    except Exception as e:
        print(f"\n✗ Error: {e}")
