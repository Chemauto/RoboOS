#!/usr/bin/env python3
import sys
import time
sys.path.insert(0, '/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla')
from utils.udp_client import UDPClient

client = UDPClient(host="127.0.0.1", port=23456)

print("=== 测试车辆控制 ===\n")

print("1. 停止车辆...")
client.send_control(steer=0.0, throttle=0.0, brake=1.0)
time.sleep(2)

print("2. 前进 (throttle=0.3)...")
client.send_control(steer=0.0, throttle=0.3, brake=0.0)
time.sleep(3)

print("3. 右转 (steer=0.5)...")
client.send_control(steer=0.5, throttle=0.3, brake=0.0)
time.sleep(2)

print("4. 左转 (steer=-0.5)...")
client.send_control(steer=-0.5, throttle=0.3, brake=0.0)
time.sleep(2)

print("5. 停止...")
client.send_control(steer=0.0, throttle=0.0, brake=1.0)

print("\n✓ 测试完成!")
client.close()
