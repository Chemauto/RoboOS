#!/usr/bin/env python3
"""
证据收集脚本 - 捕获RoboOS到CARLA的完整指令流
优化版本: 解决端口冲突、重复检测、状态追踪问题
"""

import subprocess
import time
import json
from datetime import datetime
import os
import socket

class EvidenceCollector:
    def __init__(self):
        self.evidence_file = f"/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla_test/logs/roboos_carla_evidence_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.evidence = []
        self.sensor_socket = None
        self.latest_vehicle_state = None
        self.setup_sensor_listener()

    def log(self, category, message, data=None):
        """记录证据"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        entry = {
            "timestamp": timestamp,
            "category": category,
            "message": message,
            "data": data
        }
        self.evidence.append(entry)

        # 实时打印
        print(f"\n{'='*70}")
        print(f"⏰ {timestamp}")
        print(f"📁 类别: {category}")
        print(f"📝 {message}")
        if data:
            print(f"📊 数据: {json.dumps(data, ensure_ascii=False, indent=2)}")
        print('='*70)

        # 写入文件
        with open(self.evidence_file, 'a', encoding='utf-8') as f:
            f.write(f"\n{'='*70}\n")
            f.write(f"时间: {timestamp}\n")
            f.write(f"类别: {category}\n")
            f.write(f"消息: {message}\n")
            if data:
                f.write(f"数据: {json.dumps(data, ensure_ascii=False, indent=2)}\n")
            f.write('='*70 + '\n')

    def setup_sensor_listener(self):
        """设置传感器数据监听 - 优化端口绑定"""
        ports_to_try = [12345, 12346, 12347, 12348]

        for port in ports_to_try:
            try:
                self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sensor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sensor_socket.settimeout(0.1)
                self.sensor_socket.bind(('127.0.0.1', port))
                self.log("传感器监听", f"✅ 成功绑定传感器端口: {port}")
                return
            except OSError as e:
                if self.sensor_socket:
                    self.sensor_socket.close()
                    self.sensor_socket = None
                if port == ports_to_try[-1]:
                    self.log("传感器监听", f"⚠️  所有端口均被占用,无法监听传感器数据")
                continue
            except Exception as e:
                self.log("传感器监听", f"⚠️  传感器监听设置失败: {e}")
                self.sensor_socket = None
                return

    def get_vehicle_state(self):
        """获取当前车辆状态"""
        if not self.sensor_socket:
            return None

        try:
            data, _ = self.sensor_socket.recvfrom(4096)
            state = json.loads(data.decode('utf-8'))
            self.latest_vehicle_state = state
            return state
        except socket.timeout:
            return self.latest_vehicle_state
        except Exception as e:
            return None

    def check_slaver_running(self):
        """检查slaver是否运行"""
        try:
            result = subprocess.run(['pgrep', '-f', 'run.py'], capture_output=True, text=True)
            if result.returncode != 0:
                self.log("系统检查", "❌ 未找到run.py进程")
                return False

            for pid in result.stdout.strip().split('\n'):
                if pid:
                    try:
                        cwd = os.readlink(f'/proc/{pid}/cwd')
                        if 'slaver' in cwd:
                            self.log("系统检查", f"✅ Slaver进程正在运行 (PID: {pid}, 目录: {cwd})")
                            return True
                    except:
                        continue

            self.log("系统检查", "❌ Slaver进程未运行")
            return False
        except Exception as e:
            self.log("系统检查", f"❌ 检查失败: {e}")
            return False

    def check_carla_port(self):
        """检查CARLA端口"""
        try:
            result = subprocess.run(['netstat', '-an'], capture_output=True, text=True)
            if '23456' in result.stdout:
                self.log("系统检查", "✅ CARLA UDP端口23456在监听")
                return True
            else:
                self.log("系统检查", "⚠️  CARLA端口未检测到")
                return False
        except Exception as e:
            self.log("系统检查", f"⚠️  端口检查失败: {e}")
            return False

    def extract_command_info(self, line):
        """提取指令信息,用于去重"""
        try:
            # 尝试解析JSON格式的指令
            if '{"name":' in line or "{\\\"name\\\":" in line:
                # 处理转义的JSON
                line_clean = line.replace('\\"', '"')
                start = line_clean.find('{"name":')
                if start != -1:
                    json_str = line_clean[start:start+200]
                    # 找到完整的JSON
                    brace_count = 0
                    end = start
                    for i, char in enumerate(json_str):
                        if char == '{':
                            brace_count += 1
                        elif char == '}':
                            brace_count -= 1
                            if brace_count == 0:
                                end = i + 1
                                break
                    if end > start:
                        cmd_json = json.loads(json_str[:end])
                        return f"{cmd_json.get('name')}:{json.dumps(cmd_json.get('arguments', {}))}"

            # 处理中文描述格式
            for keyword in ['move_forward', 'turn_vehicle', 'stop_vehicle', 'get_vehicle_state', 'get_gnss_data']:
                if keyword in line:
                    # 提取参数信息
                    if '速度' in line or 'speed' in line:
                        return f"{keyword}:with_params"
                    elif '角度' in line or 'angle' in line:
                        return f"{keyword}:with_params"
                    else:
                        return f"{keyword}:no_params"
        except:
            pass

        return line.strip()[:100]  # 返回前100个字符作为标识

    def monitor_slaver_log(self):
        """监控slaver日志 - 持续监控直到Ctrl+C"""
        log_file = "/home/dora/RoboOS/slaver/.log/agent.log"

        if not os.path.exists(log_file):
            self.log("日志监控", f"❌ 日志文件不存在: {log_file}")
            return

        self.log("日志监控", f"🔍 开始监控日志文件: {log_file}")
        self.log("日志监控", "⏸️  按 Ctrl+C 停止监控")

        # 获取当前文件大小
        initial_size = os.path.getsize(log_file)
        last_size = initial_size

        # 记录初始车辆状态
        initial_state = self.get_vehicle_state()
        if initial_state:
            self.log("车辆状态", "📍 初始车辆状态", initial_state)
        elif not self.sensor_socket:
            self.log("车辆状态", "⚠️  传感器端口未绑定,无法获取车辆状态")

        command_count = 0
        last_command_time = 0

        try:
            while True:
                current_size = os.path.getsize(log_file)

                if current_size > last_size:
                    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                        f.seek(last_size)
                        new_content = f.read()

                        for line in new_content.split('\n'):
                            if any(keyword in line for keyword in ['move_forward', 'turn_vehicle', 'stop_vehicle', 'get_vehicle_state', 'get_gnss_data']):

                                # 提取指令标识用于去重
                                command_hash = self.extract_command_info(line)
                                current_time = time.time()

                                # 去重逻辑: 相同指令在2秒内只记录一次
                                if command_hash != self.last_command_hash or (current_time - last_command_time) > 2.0:
                                    command_count += 1
                                    self.last_command_hash = command_hash
                                    last_command_time = current_time

                                    self.log("指令捕获", f"🎯 检测到车辆控制指令 (#{command_count})", {"log": line.strip()})

                                    # 等待指令执行
                                    time.sleep(0.3)

                                    # 获取执行后的车辆状态
                                    after_state = self.get_vehicle_state()
                                    if after_state:
                                        self.log("车辆状态", "📍 指令执行后车辆状态", after_state)

                    last_size = current_size

                time.sleep(0.5)

        except KeyboardInterrupt:
            self.log("日志监控", "⚠️  用户中断监控 (Ctrl+C)")
        finally:
            if self.sensor_socket:
                self.sensor_socket.close()

    def generate_report(self):
        """生成证据报告"""
        print("\n" + "="*70)
        print("📊 证据收集报告")
        print("="*70)
        print(f"证据文件: {self.evidence_file}")
        print(f"总证据数: {len(self.evidence)}")
        print("\n证据摘要:")

        categories = {}
        for entry in self.evidence:
            cat = entry['category']
            categories[cat] = categories.get(cat, 0) + 1

        for cat, count in categories.items():
            print(f"  - {cat}: {count}条")

        print("\n" + "="*70)
        print(f"✅ 完整证据已保存到: {self.evidence_file}")
        print("="*70)

def main():
    print("""
╔══════════════════════════════════════════════════════════════════╗
║          RoboOS → CARLA 证据收集系统 (优化版)                   ║
║          实时监控并记录完整的指令执行证据                        ║
╚══════════════════════════════════════════════════════════════════╝
    """)

    collector = EvidenceCollector()

    # 系统检查
    print("\n🔍 执行系统检查...")
    collector.check_slaver_running()
    collector.check_carla_port()

    print("\n" + "="*70)
    print("✅ 证据收集系统已启动")
    print("="*70)
    print("\n📋 请在RoboOS UI界面发送测试指令:")
    print("   1. '获取车辆状态'")
    print("   2. '让车辆前进,速度3米每秒'")
    print("   3. '停止车辆'")
    print("   4. '右转80度'")
    print("\n⏸️  监控持续运行中,按 Ctrl+C 停止")
    print("="*70 + "\n")

    # 开始监控
    try:
        collector.monitor_slaver_log()
    except KeyboardInterrupt:
        print("\n⚠️  监控被中断")

    # 生成报告
    collector.generate_report()

if __name__ == "__main__":
    main()
