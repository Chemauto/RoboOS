#!/usr/bin/env python3
"""
测试传感器数据接收
检查是否能从CARLA接收到GNSS和IMU数据
"""
import socket
import json
import time

def test_sensor_data(port=12347, timeout=5):
    """测试从指定端口接收传感器数据"""
    print(f"[测试] 尝试从端口 {port} 接收传感器数据...")
    print(f"[测试] 超时时间: {timeout}秒\n")

    try:
        # 创建UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)

        # 尝试绑定端口
        try:
            sock.bind(("0.0.0.0", port))
            print(f"✅ 成功绑定端口 {port}")
        except OSError as e:
            print(f"⚠️  端口 {port} 已被占用: {e}")
            print(f"⚠️  尝试不绑定直接接收...\n")

        # 接收数据
        print(f"[等待] 正在监听传感器数据...\n")

        gnss_received = False
        imu_received = False
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(4096)
                msg = json.loads(data.decode('utf-8'))

                if msg.get("id") == "gnss":
                    if not gnss_received:
                        print(f"✅ 接收到GNSS数据:")
                        print(f"   来源: {addr}")
                        print(f"   位置: x={msg.get('x', 0):.6f}, y={msg.get('y', 0):.6f}, z={msg.get('z', 0):.6f}")
                        gnss_received = True

                elif msg.get("id") == "imu":
                    if not imu_received:
                        print(f"✅ 接收到IMU数据:")
                        print(f"   来源: {addr}")
                        print(f"   朝向: {msg.get('heading_deg', 0):.2f}°")
                        accel = msg.get('accelerometer', {})
                        print(f"   加速度: x={accel.get('x', 0):.2f}, y={accel.get('y', 0):.2f}, z={accel.get('z', 0):.2f}")
                        imu_received = True

                # 如果两种数据都收到了,退出
                if gnss_received and imu_received:
                    print(f"\n✅ 测试成功! 两种传感器数据都已接收")
                    break

            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                print(f"⚠️  JSON解析错误: {e}")
            except Exception as e:
                print(f"❌ 接收错误: {e}")
                break

        # 超时检查
        if not gnss_received and not imu_received:
            print(f"\n❌ 测试失败: {timeout}秒内未接收到任何传感器数据")
            print(f"\n可能原因:")
            print(f"1. simple_vehicle_control.py 未运行")
            print(f"2. CARLA未发送传感器数据")
            print(f"3. 端口配置错误")
            return False
        elif not gnss_received:
            print(f"\n⚠️  部分成功: 只接收到IMU数据,未接收到GNSS数据")
            return False
        elif not imu_received:
            print(f"\n⚠️  部分成功: 只接收到GNSS数据,未接收到IMU数据")
            return False

        return True

    except Exception as e:
        print(f"❌ 测试异常: {e}")
        return False
    finally:
        sock.close()

if __name__ == "__main__":
    print("="*60)
    print("传感器数据接收测试")
    print("="*60 + "\n")

    # 测试端口12347
    success = test_sensor_data(port=12347, timeout=5)

    print("\n" + "="*60)
    if success:
        print("结论: ✅ 传感器数据接收正常")
    else:
        print("结论: ❌ 传感器数据接收异常")
    print("="*60)
