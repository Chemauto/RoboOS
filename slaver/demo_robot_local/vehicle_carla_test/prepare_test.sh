#!/bin/bash
# RoboOS到CARLA测试准备脚本

echo "╔══════════════════════════════════════════════════════════╗"
echo "║     RoboOS → CARLA 通信测试准备                          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# 检查Redis
echo "1️⃣  检查Redis..."
if redis-cli ping > /dev/null 2>&1; then
    echo "   ✅ Redis运行正常"
else
    echo "   ❌ Redis未运行,请启动Redis"
    exit 1
fi

# 检查CARLA端口
echo ""
echo "2️⃣  检查CARLA UDP端口..."
if netstat -an 2>/dev/null | grep -q "23456"; then
    echo "   ✅ CARLA端口23456在监听"
else
    echo "   ⚠️  未检测到端口23456,请确认CARLA和Leaderboard已启动"
fi

# 检查slaver配置
echo ""
echo "3️⃣  检查Slaver配置..."
if [ -f "slaver/demo_robot_local/skill.py" ]; then
    echo "   ✅ skill.py存在"

    # 检查vehicle模块导入
    if grep -q "vehicle_simulation" slaver/demo_robot_local/skill.py; then
        echo "   ✅ vehicle模块已导入"
    else
        echo "   ❌ vehicle模块未导入"
        exit 1
    fi
else
    echo "   ❌ skill.py不存在"
    exit 1
fi

# 创建日志监控脚本
echo ""
echo "4️⃣  创建日志监控..."
cat > /tmp/monitor_slaver_log.sh << 'EOF'
#!/bin/bash
echo "🔍 开始监控Slaver日志..."
echo "监控文件: slaver/.log/agent.log"
echo "="
tail -f slaver/.log/agent.log | grep --line-buffered -E "vehicle_|control|sensor|simulation" | while read line; do
    echo "[$(date '+%H:%M:%S')] $line"
done
EOF
chmod +x /tmp/monitor_slaver_log.sh

echo "   ✅ 日志监控脚本已创建: /tmp/monitor_slaver_log.sh"

# 创建UDP监控脚本
echo ""
echo "5️⃣  创建UDP监控..."
cat > /tmp/monitor_udp.py << 'EOF'
#!/usr/bin/env python3
import socket
import json
import sys
from datetime import datetime

print("🎧 监听发送到CARLA的UDP命令 (127.0.0.1:23456)")
print("="*60)

# 注意: 这个脚本需要在发送命令的同时运行tcpdump或使用端口镜像
# 简化版本: 直接监听一个测试端口
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0)

try:
    # 尝试绑定到23456端口(如果CARLA未占用)
    sock.bind(('127.0.0.1', 23456))
    print("✅ 成功绑定到23456端口")
except:
    print("⚠️  端口23456已被占用(CARLA正在使用)")
    print("   使用tcpdump监控: sudo tcpdump -i lo -n udp port 23456 -A")
    sys.exit(0)

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[{timestamp}] 📨 收到控制命令:")
            print(f"  来源: {addr}")
            print(f"  内容: {json.dumps(msg, indent=2)}")
            print("="*60)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break
except Exception as e:
    print(f"❌ 监控错误: {e}")
finally:
    sock.close()
EOF
chmod +x /tmp/monitor_udp.py

echo "   ✅ UDP监控脚本已创建: /tmp/monitor_udp.py"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║     ✅ 测试准备完成                                      ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "📋 下一步操作:"
echo ""
echo "1. 在新终端启动日志监控:"
echo "   cd /home/dora/RoboOS"
echo "   /tmp/monitor_slaver_log.sh"
echo ""
echo "2. 确保RoboOS四个组件已启动:"
echo "   ./act-four-terminal.sh"
echo ""
echo "3. 打开RoboOS UI界面"
echo ""
echo "4. 发送测试指令:"
echo "   - '获取车辆状态'"
echo "   - '让车辆前进,速度3米每秒'"
echo "   - '停止车辆'"
echo ""
echo "5. 观察:"
echo "   - 日志监控终端的输出"
echo "   - CARLA画面中车辆的响应"
echo ""
echo "="*60
