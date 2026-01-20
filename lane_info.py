#!/usr/bin/env python3
"""
车道信息显示脚本
持续显示车辆当前所在车道的信息
"""
import sys
sys.path.append('/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')

import carla
import time

def get_lane_change_str(lane_change):
    """转换车道变更枚举为可读字符串"""
    if lane_change == carla.LaneChange.NONE:
        return "不可变道"
    elif lane_change == carla.LaneChange.Right:
        return "可右变道"
    elif lane_change == carla.LaneChange.Left:
        return "可左变道"
    elif lane_change == carla.LaneChange.Both:
        return "可左右变道"
    return "未知"

def get_lane_type_str(lane_type):
    """转换车道类型枚举为可读字符串"""
    if lane_type == carla.LaneType.Driving:
        return "行车道"
    elif lane_type == carla.LaneType.Sidewalk:
        return "人行道"
    elif lane_type == carla.LaneType.Shoulder:
        return "路肩"
    elif lane_type == carla.LaneType.Biking:
        return "自行车道"
    elif lane_type == carla.LaneType.Parking:
        return "停车区"
    return "其他"

def main():
    print("[车道信息] 启动中...")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    print("[车道信息] ✓ 已连接到CARLA\n")

    try:
        while True:
            vehicles = world.get_actors().filter('vehicle.*')

            if len(vehicles) > 0:
                vehicle = vehicles[0]
                location = vehicle.get_location()
                waypoint = carla_map.get_waypoint(location)

                print("\n" + "="*60)
                print(f"车辆位置: x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f}")
                print(f"车道ID: {waypoint.lane_id}")
                print(f"道路ID: {waypoint.road_id}")
                print(f"车道类型: {get_lane_type_str(waypoint.lane_type)}")
                print(f"车道宽度: {waypoint.lane_width:.2f}m")
                print(f"车道变更: {get_lane_change_str(waypoint.lane_change)}")
                print(f"是否路口: {'是' if waypoint.is_junction else '否'}")
                print("="*60)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[车道信息] 已停止")

if __name__ == "__main__":
    main()
