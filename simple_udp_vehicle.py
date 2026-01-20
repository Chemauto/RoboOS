#!/usr/bin/env python3
"""简化的CARLA车辆UDP控制"""
import sys
sys.path.append('/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')

import carla
import socket
import json
import threading
import time

UDP_PORT = 23456
control_command = {"steer": 0.0, "throttle": 0.0, "brake": 1.0, "reverse": False}
control_lock = threading.Lock()

def receive_control_loop():
    """接收UDP控制指令"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    print(f"[UDP] 正在监听端口 {UDP_PORT}...", flush=True)

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            if msg.get("id") == "control":
                with control_lock:
                    control_command["steer"] = float(msg.get("steer", 0.0))
                    control_command["throttle"] = float(msg.get("throttle", 0.0))
                    control_command["brake"] = float(msg.get("brake", 0.0))
                    control_command["reverse"] = bool(msg.get("reverse", False))
                    print(f"[控制] Steer:{control_command['steer']:.2f} Throttle:{control_command['throttle']:.2f} Brake:{control_command['brake']:.2f}", flush=True)
        except Exception as e:
            print(f"[UDP错误] {e}", flush=True)

def main():
    print("[CARLA] 连接中...", flush=True)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    print("[CARLA] ✓ 已连接", flush=True)

    # 获取现有车辆
    vehicles = world.get_actors().filter('vehicle.*')
    if len(vehicles) == 0:
        print("[错误] 没有找到车辆!", flush=True)
        return

    vehicle = vehicles[0]
    print(f"[车辆] 使用车辆 ID: {vehicle.id}", flush=True)

    # 启动UDP接收线程
    threading.Thread(target=receive_control_loop, daemon=True).start()

    print("[系统] ✓ 就绪!", flush=True)

    # 主控制循环
    try:
        while True:
            with control_lock:
                control = carla.VehicleControl()
                control.steer = control_command["steer"]
                control.throttle = control_command["throttle"]
                control.brake = control_command["brake"]
                control.reverse = control_command["reverse"]
                vehicle.apply_control(control)
            time.sleep(0.05)  # 20Hz
    except KeyboardInterrupt:
        print("\n[停止] 清理中...", flush=True)
        vehicle.apply_control(carla.VehicleControl(brake=1.0))

if __name__ == "__main__":
    main()
