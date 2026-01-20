#!/usr/bin/env python3
"""
摄像机跟随脚本
持续更新摄像机位置,跟随车辆移动
"""
import sys
sys.path.append('/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')

import carla
import math
import time

# 跟随参数
FOLLOW_DISTANCE = 6.0  # 后方距离(米)
FOLLOW_HEIGHT = 3.0    # 高度(米)
PITCH_ANGLE = -15      # 俯角(度)
UPDATE_RATE = 0.1      # 更新频率(秒)

def main():
    print("[摄像机跟随] 启动中...")

    # 连接CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    print("[摄像机跟随] ✓ 已连接到CARLA")

    spectator = world.get_spectator()

    try:
        while True:
            # 查找车辆
            vehicles = world.get_actors().filter('vehicle.*')

            if len(vehicles) > 0:
                vehicle = vehicles[0]  # 跟随第一辆车

                # 获取车辆位置和朝向
                v_transform = vehicle.get_transform()
                v_loc = v_transform.location
                v_rot = v_transform.rotation

                # 计算摄像机位置(车辆后方)
                yaw_rad = math.radians(v_rot.yaw)
                camera_x = v_loc.x - FOLLOW_DISTANCE * math.cos(yaw_rad)
                camera_y = v_loc.y - FOLLOW_DISTANCE * math.sin(yaw_rad)
                camera_z = v_loc.z + FOLLOW_HEIGHT

                # 更新摄像机位置
                camera_transform = carla.Transform(
                    carla.Location(x=camera_x, y=camera_y, z=camera_z),
                    carla.Rotation(pitch=PITCH_ANGLE, yaw=v_rot.yaw, roll=0)
                )
                spectator.set_transform(camera_transform)

            time.sleep(UPDATE_RATE)

    except KeyboardInterrupt:
        print("\n[摄像机跟随] 已停止")

if __name__ == "__main__":
    main()
