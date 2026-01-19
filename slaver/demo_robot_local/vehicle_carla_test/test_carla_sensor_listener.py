#!/usr/bin/env python3
"""
监听CARLA发送的传感器数据,验证CARLA是否在运行并发送数据
"""

import socket
import json
import time
import sys

def listen_carla_sensor_data(duration=10):
    """监听CARLA发送的传感器数据"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        sock.bind(('192.168.1.101', 12345))

        print(f"🎧 开始监听CARLA传感器数据 (192.168.1.101:12345)")
        print(f"   监听时长: {duration}秒")
        print("=" * 60)

        start_time = time.time()
        received_count = 0
        gnss_count = 0
        imu_count = 0

        while time.time() - start_time < duration:
            try:
                data, addr = sock.recvfrom(4096)
                msg = json.loads(data.decode('utf-8'))
                received_count += 1

                if msg.get("id") == "gnss":
                    gnss_count += 1
                    x, y, z = msg.get("x", 0), msg.get("y", 0), msg.get("z", 0)
                    print(f"📍 GNSS数据 #{gnss_count}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

                elif msg.get("id") == "imu":
                    imu_count += 1
                    heading = msg.get("heading_deg", 0)
                    print(f"🧭 IMU数据 #{imu_count}: heading={heading:.2f}°")

            except socket.timeout:
                print("⏳ 等待数据...")
                continue
            except json.JSONDecodeError as e:
                print(f"⚠️  JSON解析错误: {e}")
                continue

        sock.close()

        print("=" * 60)
        print(f"✅ 监听完成")
        print(f"   总接收: {received_count}条消息")
        print(f"   GNSS: {gnss_count}条")
        print(f"   IMU: {imu_count}条")

        if received_count > 0:
            print("\n✅ CARLA正在发送传感器数据")
            return True
        else:
            print("\n❌ 未收到CARLA传感器数据")
            print("   可能原因:")
            print("   1. CARLA未启动")
            print("   2. Leaderboard未运行")
            print("   3. dora.py agent未配置传感器数据发送")
            return False

    except Exception as e:
        print(f"❌ 监听失败: {e}")
        return False

if __name__ == "__main__":
    duration = 10
    if len(sys.argv) > 1:
        duration = int(sys.argv[1])

    listen_carla_sensor_data(duration)
