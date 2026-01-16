#!/usr/bin/env python3
import sys
import time
sys.path.insert(0, '/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla')
from utils.udp_client import UDPClient

client = UDPClient(host="127.0.0.1", port=23456)
print("Sending forward command...")
client.send_control(steer=0.0, throttle=0.5, brake=0.0)
time.sleep(0.5)
print("Done")
client.close()
