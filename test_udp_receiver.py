#!/usr/bin/env python3
"""简单的UDP接收测试脚本"""
import socket
import json

UDP_PORT = 23456

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
print(f"[UDP测试] 正在监听端口 {UDP_PORT}...")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        msg = json.loads(data.decode('utf-8'))
        print(f"[UDP测试] 收到消息: {msg}")
    except Exception as e:
        print(f"[UDP测试] 错误: {e}")
