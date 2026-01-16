#!/usr/bin/env python3
"""
获取Carla车辆的位置和朝向信息
"""
import carla
import time

def get_vehicle_info():
    try:
        # 连接到Carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)

        # 获取世界和车辆
        world = client.get_world()
        all_actors = world.get_actors()

        # 尝试多种方式查找车辆
        vehicles = all_actors.filter('vehicle.*')
        hero_vehicles = [a for a in all_actors if 'hero' in a.type_id.lower() or 'hero' in str(a.attributes.get('role_name', '')).lower()]

        print(f"总actors数量: {len(all_actors)}")
        print(f"vehicle.*匹配数量: {len(vehicles)}")
        print(f"hero车辆数量: {len(hero_vehicles)}")

        # 优先使用hero车辆
        if hero_vehicles:
            vehicle = hero_vehicles[0]
        elif vehicles:
            vehicle = vehicles[0]
        else:
            print("❌ 没有找到车辆")
            print("尝试列出包含vehicle的actors:")
            for actor in all_actors:
                if 'vehicle' in actor.type_id.lower():
                    print(f"  - {actor.type_id} (role: {actor.attributes.get('role_name', 'N/A')})")
            return
        transform = vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation
        velocity = vehicle.get_velocity()

        print("=" * 50)
        print(f"🚗 车辆信息 (时间: {time.strftime('%H:%M:%S')})")
        print("=" * 50)
        print(f"📍 位置坐标:")
        print(f"   X: {location.x:.2f} m")
        print(f"   Y: {location.y:.2f} m")
        print(f"   Z: {location.z:.2f} m")
        print(f"\n🧭 朝向角度:")
        print(f"   Pitch: {rotation.pitch:.2f}°")
        print(f"   Yaw:   {rotation.yaw:.2f}°")
        print(f"   Roll:  {rotation.roll:.2f}°")
        print(f"\n⚡ 速度:")
        print(f"   Vx: {velocity.x:.2f} m/s")
        print(f"   Vy: {velocity.y:.2f} m/s")
        print(f"   Vz: {velocity.z:.2f} m/s")
        print(f"   总速度: {(velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5:.2f} m/s")
        print("=" * 50)

    except Exception as e:
        print(f"❌ 错误: {e}")

if __name__ == "__main__":
    get_vehicle_info()
