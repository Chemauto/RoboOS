#!/usr/bin/env python3
"""检查CARLA中的车辆和传感器"""
import sys
sys.path.append('/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')

import carla

try:
    # 连接CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    
    print("="*60)
    print("CARLA车辆和传感器检查")
    print("="*60 + "\n")
    
    # 检查车辆
    vehicles = world.get_actors().filter('vehicle.*')
    print(f"✅ 找到 {len(vehicles)} 辆车辆\n")
    
    if len(vehicles) == 0:
        print("❌ 没有车辆! 需要先生成车辆")
        sys.exit(1)
    
    # 检查每辆车的传感器
    for i, vehicle in enumerate(vehicles):
        print(f"车辆 #{i+1}:")
        print(f"  ID: {vehicle.id}")
        print(f"  类型: {vehicle.type_id}")
        
        location = vehicle.get_location()
        print(f"  位置: ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})")
        
        # 查找附加到该车辆的传感器
        all_actors = world.get_actors()
        sensors = []
        
        for actor in all_actors:
            if actor.parent and actor.parent.id == vehicle.id:
                if 'sensor' in actor.type_id:
                    sensors.append(actor)
        
        print(f"  传感器数量: {len(sensors)}")
        
        if len(sensors) > 0:
            print(f"  传感器列表:")
            for sensor in sensors:
                print(f"    - {sensor.type_id} (ID: {sensor.id})")
        else:
            print(f"  ⚠️  未找到传感器!")
        
        print()
    
    print("="*60)
    if len(vehicles) > 0 and len(sensors) > 0:
        print("结论: ✅ 车辆和传感器都已就绪")
    elif len(vehicles) > 0:
        print("结论: ⚠️  车辆已生成,但未添加传感器")
    else:
        print("结论: ❌ 未找到车辆")
    print("="*60)
    
except Exception as e:
    print(f"❌ 错误: {e}")
    sys.exit(1)
