#!/usr/bin/env python3
"""Test script for RoboOS to CARLA communication"""
import sys
import time
sys.path.insert(0, '/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla')

from utils.udp_client import UDPClient

def test_basic_control():
    """Test basic vehicle control commands"""
    print("=== Testing RoboOS to CARLA Communication ===\n")

    # Initialize UDP client
    print("1. Initializing UDP client...")
    client = UDPClient(host="192.168.1.1", port=23456)
    print(f"   Connected to 192.168.1.1:23456\n")

    # Test 1: Stop vehicle
    print("2. Test: Stop vehicle (brake=1.0)")
    success = client.send_control(steer=0.0, throttle=0.0, brake=1.0)
    print(f"   Result: {'✓ Success' if success else '✗ Failed'}\n")
    time.sleep(2)

    # Test 2: Move forward
    print("3. Test: Move forward (throttle=0.3)")
    success = client.send_control(steer=0.0, throttle=0.3, brake=0.0)
    print(f"   Result: {'✓ Success' if success else '✗ Failed'}\n")
    time.sleep(3)

    # Test 3: Turn right
    print("4. Test: Turn right (steer=0.5, throttle=0.3)")
    success = client.send_control(steer=0.5, throttle=0.3, brake=0.0)
    print(f"   Result: {'✓ Success' if success else '✗ Failed'}\n")
    time.sleep(2)

    # Test 4: Turn left
    print("5. Test: Turn left (steer=-0.5, throttle=0.3)")
    success = client.send_control(steer=-0.5, throttle=0.3, brake=0.0)
    print(f"   Result: {'✓ Success' if success else '✗ Failed'}\n")
    time.sleep(2)

    # Test 5: Emergency brake
    print("6. Test: Emergency brake (brake=1.0)")
    success = client.send_control(steer=0.0, throttle=0.0, brake=1.0)
    print(f"   Result: {'✓ Success' if success else '✗ Failed'}\n")

    client.close()
    print("=== Test Complete ===")
    print("\nNote: Check CARLA simulator to verify vehicle movements")

if __name__ == "__main__":
    try:
        test_basic_control()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
