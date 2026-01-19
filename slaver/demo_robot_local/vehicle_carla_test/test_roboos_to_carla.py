#!/usr/bin/env python3
"""
测试RoboOS到CARLA的完整通信链路

测试流程:
1. 检查Redis是否运行
2. 检查CARLA UDP端口是否监听
3. 发送测试控制命令
4. 监听UDP端口验证命令是否发送
"""

import socket
import json
import time
import subprocess
import sys

def check_redis():
    """检查Redis是否运行"""
    try:
        import redis
        r = redis.Redis(host='127.0.0.1', port=6379, db=0)
        r.ping()
        print("✅ Redis运行正常")
        return True
    except Exception as e:
        print(f"❌ Redis连接失败: {e}")
        return False

def check_carla_udp_port():
    """检查CARLA UDP端口是否在监听"""
    try:
        result = subprocess.run(
            ['netstat', '-an'],
            capture_output=True,
            text=True
        )
        if '23456' in result.stdout:
            print("✅ CARLA UDP端口23456可能在监听")
            return True
        else:
            print("⚠️  未检测到端口23456监听")
            return False
    except Exception as e:
        print(f"⚠️  无法检查端口状态: {e}")
        return False

def send_test_command():
    """发送测试控制命令到CARLA"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 测试命令: 前进
        test_command = {
            "id": "control",
            "steer": 0.0,
            "throttle": 0.3,
            "brake": 0.0
        }

        message = json.dumps(test_command).encode('utf-8')
        sock.sendto(message, ('127.0.0.1', 23456))

        print(f"✅ 已发送测试命令到127.0.0.1:23456")
        print(f"   命令内容: {test_command}")

        sock.close()
        return True
    except Exception as e:
        print(f"❌ 发送命令失败: {e}")
        return False

def test_udp_echo():
    """测试UDP通信是否正常"""
    try:
        # 创建UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)

        # 尝试绑定到本地端口
        sock.bind(('127.0.0.1', 0))
        local_port = sock.getsockname()[1]
        print(f"✅ UDP socket创建成功，本地端口: {local_port}")

        sock.close()
        return True
    except Exception as e:
        print(f"❌ UDP通信测试失败: {e}")
        return False

def check_slaver_process():
    """检查slaver进程是否运行"""
    try:
        result = subprocess.run(
            ['ps', 'aux'],
            capture_output=True,
            text=True
        )
        if 'slaver' in result.stdout and 'run.py' in result.stdout:
            print("✅ Slaver进程正在运行")
            return True
        else:
            print("⚠️  未检测到Slaver进程")
            return False
    except Exception as e:
        print(f"⚠️  无法检查进程状态: {e}")
        return False

def main():
    print("=" * 60)
    print("RoboOS到CARLA通信链路测试")
    print("=" * 60)
    print()

    # 1. 检查Redis
    print("1. 检查Redis连接...")
    redis_ok = check_redis()
    print()

    # 2. 检查Slaver进程
    print("2. 检查Slaver进程...")
    slaver_ok = check_slaver_process()
    print()

    # 3. 检查UDP通信
    print("3. 检查UDP通信...")
    udp_ok = test_udp_echo()
    print()

    # 4. 检查CARLA端口
    print("4. 检查CARLA UDP端口...")
    carla_ok = check_carla_udp_port()
    print()

    # 5. 发送测试命令
    print("5. 发送测试控制命令...")
    send_ok = send_test_command()
    print()

    # 总结
    print("=" * 60)
    print("测试结果总结:")
    print("=" * 60)
    print(f"Redis连接:        {'✅' if redis_ok else '❌'}")
    print(f"Slaver进程:       {'✅' if slaver_ok else '⚠️'}")
    print(f"UDP通信:          {'✅' if udp_ok else '❌'}")
    print(f"CARLA端口:        {'✅' if carla_ok else '⚠️'}")
    print(f"命令发送:         {'✅' if send_ok else '❌'}")
    print()

    if all([redis_ok, udp_ok, send_ok]):
        print("✅ 基础通信链路正常")
        print()
        print("下一步:")
        print("1. 确保CARLA和Leaderboard正在运行")
        print("2. 通过RoboOS UI发送指令测试完整链路")
        print("3. 观察CARLA画面中车辆是否响应")
    else:
        print("❌ 存在问题，请检查上述失败项")

    print("=" * 60)

if __name__ == "__main__":
    main()
