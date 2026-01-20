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
import math

# 网络配置
UDP_IP = "127.0.0.1"
UDP_CONTROL_IP = "0.0.0.0"
UDP_PORT_GNSS_IMU = 12347  # 改用 12347 避免与 VehicleController 冲突
UDP_PORT_CONTROL = 23456

# 全局变量
control_command = {"steer": 0.0, "throttle": 0.0, "brake": 1.0, "reverse": False}
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
                    control_command["reverse"] = bool(msg.get("reverse", False))
                    print(f"[控制指令] Steer: {control_command['steer']:.2f}, "
                          f"Throttle: {control_command['throttle']:.2f}, "
                          f"Brake: {control_command['brake']:.2f}, "
                          f"Reverse: {control_command['reverse']}")
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

    # 使用当前地图,不切换场景
    print(f"[CARLA] ✓ 使用当前地图: {current_map}")

    # 清理旧车辆
    print("[清理] 检查并删除旧车辆...")
    existing_vehicles = world.get_actors().filter('vehicle.*')
    if len(existing_vehicles) > 0:
        for old_vehicle in existing_vehicles:
            old_vehicle.destroy()
            print(f"[清理] ✓ 已删除旧车辆 (ID: {old_vehicle.id})")
    else:
        print("[清理] 无旧车辆")

    # 获取蓝图库
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    # 获取spawn点和地图
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if not spawn_points:
        print("[错误] 没有可用的spawn点")
        return

    # Spawn车辆 - 使用固定朝向,并居中到车道中间
    spawn_point = spawn_points[0]
    original_z = spawn_point.location.z  # 保存原始z坐标

    # 获取该位置的车道中心点
    waypoint = carla_map.get_waypoint(spawn_point.location)

    # 使用车道中心的x,y坐标,保持原始z坐标
    spawn_point.location = carla.Location(
        x=waypoint.transform.location.x,
        y=waypoint.transform.location.y,
        z=original_z
    )

    # 设置固定的朝向角度 (yaw=0表示正东方向)
    FIXED_YAW = 180.0  # 180度=正西方向
    spawn_point.rotation.yaw = FIXED_YAW
    spawn_point.rotation.pitch = 0.0
    spawn_point.rotation.roll = 0.0

    print(f"[车辆] Spawning at x={spawn_point.location.x:.2f}, "
          f"y={spawn_point.location.y:.2f}, z={spawn_point.location.z:.2f}")
    print(f"[车辆] 已居中到车道中间 (车道ID: {waypoint.lane_id})")
    print(f"[车辆] 固定朝向: Yaw={FIXED_YAW:.1f}°")
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"[车辆] ✓ 已生成 (ID: {vehicle.id})")

    # 等待车辆位置更新
    time.sleep(0.5)
    world.tick()  # 强制更新世界状态

    # 设置摄像机跟随视角
    v_transform = vehicle.get_transform()
    v_loc = v_transform.location
    v_rot = v_transform.rotation

    follow_distance = 6.0
    follow_height = 3.0
    yaw_rad = math.radians(v_rot.yaw)

    camera_x = v_loc.x - follow_distance * math.cos(yaw_rad)
    camera_y = v_loc.y - follow_distance * math.sin(yaw_rad)
    camera_z = v_loc.z + follow_height

    spectator = world.get_spectator()
    camera_transform = carla.Transform(
        carla.Location(x=camera_x, y=camera_y, z=camera_z),
        carla.Rotation(pitch=-15, yaw=v_rot.yaw, roll=0)
    )
    spectator.set_transform(camera_transform)
    print(f"[摄像机] ✓ 已设置跟随视角 (后方{follow_distance}m, 高度{follow_height}m)")
    print(f"[摄像机] 位置: ({camera_x:.2f}, {camera_y:.2f}, {camera_z:.2f})")

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

    # 车道信息显示计数器和上次车道信息
    lane_info_counter = 0
    last_lane_info = None
    camera_update_counter = 0

    # 主循环 - 应用控制指令
    try:
        while True:
            with control_lock:
                control = carla.VehicleControl()
                control.steer = control_command["steer"]
                control.throttle = control_command["throttle"]
                control.brake = control_command["brake"]
                control.reverse = control_command["reverse"]
                vehicle.apply_control(control)

            # 每2次循环更新一次摄像机位置 (0.1秒)
            camera_update_counter += 1
            if camera_update_counter >= 2:
                v_transform = vehicle.get_transform()
                v_loc = v_transform.location
                v_rot = v_transform.rotation

                yaw_rad = math.radians(v_rot.yaw)
                camera_x = v_loc.x - follow_distance * math.cos(yaw_rad)
                camera_y = v_loc.y - follow_distance * math.sin(yaw_rad)
                camera_z = v_loc.z + follow_height

                camera_transform = carla.Transform(
                    carla.Location(x=camera_x, y=camera_y, z=camera_z),
                    carla.Rotation(pitch=-15, yaw=v_rot.yaw, roll=0)
                )
                spectator.set_transform(camera_transform)
                camera_update_counter = 0

            # 每秒检查一次车道信息
            lane_info_counter += 1
            if lane_info_counter >= 20:  # 20Hz * 1秒 = 20次
                location = vehicle.get_location()
                waypoint = carla_map.get_waypoint(location)
                lane_change_str = {
                    carla.LaneChange.NONE: "不可变道",
                    carla.LaneChange.Right: "可右变道",
                    carla.LaneChange.Left: "可左变道",
                    carla.LaneChange.Both: "可左右变道"
                }.get(waypoint.lane_change, "未知")

                # 只在车道信息变化时输出
                current_lane_info = (waypoint.lane_id, waypoint.lane_width, lane_change_str)
                if current_lane_info != last_lane_info:
                    print(f"[车道] ID:{waypoint.lane_id} 宽度:{waypoint.lane_width:.1f}m {lane_change_str}")
                    last_lane_info = current_lane_info

                lane_info_counter = 0

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
