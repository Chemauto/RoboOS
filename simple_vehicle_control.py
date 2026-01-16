#!/usr/bin/env python3
"""
简单的CARLA车辆控制脚本
- 在Town04_Opt场景中spawn车辆
- 通过UDP端口23456接收控制指令
- 发送GNSS/IMU数据到UDP端口12345
- 无场景切换,持续运行
"""
import sys
sys.path.append('/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')

import carla
import socket
import json
import threading
import time

# 网络配置
UDP_IP = "127.0.0.1"
UDP_CONTROL_IP = "0.0.0.0"
UDP_PORT_GNSS_IMU = 12345
UDP_PORT_CONTROL = 23456

# 全局变量
control_command = {"steer": 0.0, "throttle": 0.0, "brake": 1.0}
control_lock = threading.Lock()
sock_gnss_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def receive_control_loop():
    """接收UDP控制指令"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_CONTROL_IP, UDP_PORT_CONTROL))
    print(f"[控制接收] 正在监听 UDP {UDP_PORT_CONTROL}...")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            if msg.get("id") == "control":
                with control_lock:
                    control_command["steer"] = float(msg.get("steer", 0.0))
                    control_command["throttle"] = float(msg.get("throttle", 0.0))
                    control_command["brake"] = float(msg.get("brake", 0.0))
                    print(f"[控制指令] Steer: {control_command['steer']:.2f}, "
                          f"Throttle: {control_command['throttle']:.2f}, "
                          f"Brake: {control_command['brake']:.2f}")
        except Exception as e:
            print(f"[控制接收错误] {e}")

def main():
    # 连接CARLA
    print("[CARLA] 连接到localhost:2000...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    current_map = world.get_map().name
    print(f"[CARLA] 当前地图: {current_map}")

    # 如果不是Town04_Opt,则加载
    if "Town04_Opt" not in current_map:
        print("[CARLA] 加载Town04_Opt...")
        client.load_world('Town04_Opt')
        world = client.get_world()
        print(f"[CARLA] ✓ 已加载: {world.get_map().name}")

    # 获取蓝图库
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    # 获取spawn点
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("[错误] 没有可用的spawn点")
        return

    # Spawn车辆
    spawn_point = spawn_points[0]
    print(f"[车辆] Spawning at x={spawn_point.location.x:.2f}, "
          f"y={spawn_point.location.y:.2f}, z={spawn_point.location.z:.2f}")
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"[车辆] ✓ 已生成 (ID: {vehicle.id})")

    # 添加GNSS传感器
    gnss_bp = blueprint_library.find('sensor.other.gnss')
    gnss_transform = carla.Transform(carla.Location(x=0, y=0, z=1.6))
    gnss_sensor = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
    print("[传感器] ✓ GNSS已添加")

    # 添加IMU传感器
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=1.6))
    imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    print("[传感器] ✓ IMU已添加")

    # GNSS回调
    def gnss_callback(data):
        msg = {
            "id": "gnss",
            "x": data.latitude,
            "y": data.longitude,
            "z": data.altitude
        }
        sock_gnss_imu.sendto(json.dumps(msg).encode('utf-8'), (UDP_IP, UDP_PORT_GNSS_IMU))

    # IMU回调
    def imu_callback(data):
        heading = (data.compass - 1.5708) % 6.2832  # 调整为正东0度
        heading_deg = (heading * 57.2958 - 90) % 360
        msg = {
            "id": "imu",
            "accelerometer": {
                "x": data.accelerometer.x,
                "y": data.accelerometer.y,
                "z": data.accelerometer.z
            },
            "gyroscope": {
                "x": data.gyroscope.x,
                "y": data.gyroscope.y,
                "z": data.gyroscope.z
            },
            "heading_deg": heading_deg
        }
        sock_gnss_imu.sendto(json.dumps(msg).encode('utf-8'), (UDP_IP, UDP_PORT_GNSS_IMU))

    gnss_sensor.listen(gnss_callback)
    imu_sensor.listen(imu_callback)

    # 启动控制接收线程
    threading.Thread(target=receive_control_loop, daemon=True).start()

    print("\n" + "="*60)
    print("✓ 系统就绪!")
    print("="*60)
    print(f"车辆ID: {vehicle.id}")
    print(f"位置: x={spawn_point.location.x:.2f}, y={spawn_point.location.y:.2f}")
    print(f"控制端口: UDP {UDP_PORT_CONTROL}")
    print(f"传感器数据端口: UDP {UDP_PORT_GNSS_IMU}")
    print("\n按 Ctrl+C 停止...")
    print("="*60 + "\n")

    # 主循环 - 应用控制指令
    try:
        while True:
            with control_lock:
                control = carla.VehicleControl()
                control.steer = control_command["steer"]
                control.throttle = control_command["throttle"]
                control.brake = control_command["brake"]
                vehicle.apply_control(control)

            time.sleep(0.05)  # 20Hz控制频率

    except KeyboardInterrupt:
        print("\n[停止] 清理资源...")
        gnss_sensor.destroy()
        imu_sensor.destroy()
        vehicle.destroy()
        sock_gnss_imu.close()
        print("[停止] ✓ 完成")

if __name__ == "__main__":
    main()
