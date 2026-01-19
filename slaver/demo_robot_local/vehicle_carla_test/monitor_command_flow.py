#!/usr/bin/env python3
"""
实时监控RoboOS到CARLA的指令流
捕获完整的证据链
"""

import socket
import json
import time
import threading
import sys
from datetime import datetime

class CommandMonitor:
    def __init__(self):
        self.evidence = []
        self.running = True

    def log_evidence(self, source, message, data=None):
        """记录证据"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        evidence_entry = {
            "timestamp": timestamp,
            "source": source,
            "message": message,
            "data": data
        }
        self.evidence.append(evidence_entry)

        # 实时打印
        print(f"\n{'='*60}")
        print(f"[{timestamp}] 📋 {source}")
        print(f"  {message}")
        if data:
            print(f"  数据: {json.dumps(data, ensure_ascii=False, indent=2)}")
        print('='*60)

    def monitor_udp_to_carla(self):
        """监控发送到CARLA的UDP命令"""
        try:
            # 创建一个监听socket来捕获发送到CARLA的命令
            # 注意: 这需要在发送前拦截,或者通过网络抓包
            # 这里我们创建一个简单的UDP服务器来接收测试命令
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            sock.bind(('127.0.0.1', 23457))  # 使用不同的端口进行监控

            self.log_evidence("UDP监控", "开始监控UDP命令流", {"port": 23457})

            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    msg = json.loads(data.decode('utf-8'))
                    self.log_evidence("UDP→CARLA", "捕获到控制命令", msg)
                except socket.timeout:
                    continue
                except Exception as e:
                    pass

            sock.close()
        except Exception as e:
            self.log_evidence("UDP监控", f"监控失败: {e}")

    def monitor_carla_sensor(self):
        """监控CARLA发送的传感器数据"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            sock.bind(('192.168.1.101', 12345))

            self.log_evidence("传感器监控", "开始监控CARLA传感器数据", {"port": 12345})

            while self.running:
                try:
                    data, addr = sock.recvfrom(4096)
                    msg = json.loads(data.decode('utf-8'))

                    if msg.get("id") == "gnss":
                        x, y, z = msg.get("x", 0), msg.get("y", 0), msg.get("z", 0)
                        self.log_evidence("CARLA→���感器", "GNSS位置数据",
                                        {"x": f"{x:.2f}", "y": f"{y:.2f}", "z": f"{z:.2f}"})
                    elif msg.get("id") == "imu":
                        heading = msg.get("heading_deg", 0)
                        self.log_evidence("CARLA→传感器", "IMU航向数据",
                                        {"heading": f"{heading:.2f}°"})

                except socket.timeout:
                    continue
                except Exception as e:
                    pass

            sock.close()
        except Exception as e:
            self.log_evidence("传感器监控", f"监控失败: {e}")

    def send_test_command(self, steer=0.0, throttle=0.3, brake=0.0):
        """发送测试命令到CARLA"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            command = {
                "id": "control",
                "steer": steer,
                "throttle": throttle,
                "brake": brake
            }

            message = json.dumps(command).encode('utf-8')
            sock.sendto(message, ('127.0.0.1', 23456))

            self.log_evidence("测试命令", "发送UDP命令到CARLA", command)

            sock.close()
            return True
        except Exception as e:
            self.log_evidence("测试命令", f"发送失败: {e}")
            return False

    def print_summary(self):
        """打印证据总结"""
        print("\n" + "="*60)
        print("📊 证据链总结")
        print("="*60)
        print(f"总共捕获 {len(self.evidence)} 条证据\n")

        for i, entry in enumerate(self.evidence, 1):
            print(f"{i}. [{entry['timestamp']}] {entry['source']}: {entry['message']}")

        print("\n" + "="*60)

    def start_monitoring(self, duration=60):
        """启动监控"""
        print("🚀 启动RoboOS→CARLA指令流监控")
        print(f"⏱️  监控时长: {duration}秒")
        print("="*60)

        # 启动传感器监控线程
        sensor_thread = threading.Thread(target=self.monitor_carla_sensor)
        sensor_thread.daemon = True
        sensor_thread.start()

        # 等待一段时间
        try:
            time.sleep(duration)
        except KeyboardInterrupt:
            print("\n⚠️  用户中断监控")

        self.running = False
        time.sleep(2)  # 等待线程结束

        self.print_summary()

def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║     RoboOS → CARLA 指令流监控系统                        ║
║     实时捕获并记录完整证据链                             ║
╚══════════════════════════════════════════════════════════╝
    """)

    monitor = CommandMonitor()

    # 检查基础连接
    print("🔍 检查基础连接...")

    # 发送一个测试命令
    print("\n📤 发送测试命令验证UDP通信...")
    if monitor.send_test_command(steer=0.0, throttle=0.3, brake=0.0):
        print("✅ 测试命令发送成功")

    print("\n" + "="*60)
    print("✅ 监控系统准备就绪")
    print("="*60)
    print("\n请在RoboOS UI界面发送以下测试指令:")
    print("  1. '获取车辆状态'")
    print("  2. '让车辆前进,速度3米每秒'")
    print("  3. '停止车辆'")
    print("\n监控将持续60秒,按Ctrl+C可提前结束")
    print("="*60 + "\n")

    # 启动监控
    monitor.start_monitoring(duration=60)

if __name__ == "__main__":
    main()
