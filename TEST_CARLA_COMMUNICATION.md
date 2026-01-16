# RoboOS到CARLA通信测试指南

**创建时间**: 2026-01-16
**目的**: 验证从RoboOS下发的指令能够到达CARLA仿真器

## 测试架构

```
RoboOS UI (Deploy)
    ↓ HTTP
RoboOS Master (任务分解)
    ↓ Redis Pub/Sub
RoboOS Slaver (调用skills)
    ↓ MCP Protocol
Vehicle CARLA Skills (skill.py)
    ↓ UDP (192.168.1.1:23456)
CARLA Simulator (dora.py agent)
```

## 前置条件

### 1. 确认CARLA仿真器正在运行
```bash
# 检查CARLA进程
ps aux | grep CarlaUE4

# 或查看日志
tail -f /tmp/carla_roboos_logs/terminal1_carla_server.log
```

### 2. 确认Leaderboard正在运行
```bash
# 检查leaderboard进程
ps aux | grep leaderboard

# 或查看日志
tail -f /tmp/carla_roboos_logs/terminal2_leaderboard.log
```

### 3. 确认网络配置
- CARLA控制命令接收地址: `192.168.1.1:23456` (UDP)
- 传感器数据发送地址: `192.168.1.101:12345` (UDP)

## 测试方法

### 方法1: 直接UDP通信测试（最简单）

**目的**: 绕过RoboOS系统，直接测试UDP通信到CARLA

**步骤**:
```bash
# 1. 确保CARLA和Leaderboard正在运行

# 2. 运行测试脚本
cd /home/dora/RoboOS
python test_carla_communication.py
```

**预期结果**:
- 脚本输出显示所有命令发送成功（✓ Success）
- CARLA仿真器中的车辆执行以下动作：
  1. 停止（制动）
  2. 前进（油门0.3）
  3. 右转前进
  4. 左转前进
  5. 紧急制动

**测试命令序列**:
1. `stop` - 制动停车
2. `move_forward` - 油门0.3前进3秒
3. `turn_right` - 右转（steer=0.5）2秒
4. `turn_left` - 左转（steer=-0.5）2秒
5. `emergency_brake` - 紧急制动

---

### 方法2: 通过RoboOS系统完整测试

**目的**: 测试完整的RoboOS → CARLA通信链路

#### 步骤1: 启动RoboOS系统

```bash
# 启动四个RoboOS组件
cd /home/dora/RoboOS
./act-four-terminal.sh
```

等待所有组件启动完成（约10-20秒）

#### 步骤2: 检查系统状态

```bash
# 检查所有进程
./check-status.sh

# 或手动检查
ps aux | grep -E "start_server|master/run|slaver/run|deploy/run"
```

#### 步骤3: 通过UI发送测试指令

打开浏览器访问RoboOS Deploy UI（通常是 `http://localhost:8000` 或配置的端口）

**测试指令示例**:

1. **基础控制测试**:
   - "让车辆停止"
   - "让车辆前进，速度5米每秒"
   - "让车辆右转30度"
   - "紧急制动"

2. **传感器数据测试**:
   - "获取车辆当前位置"
   - "获取车辆状态"
   - "获取GPS数据"

3. **组合任务测试**:
   - "让车辆前进10米然后停止"
   - "让车辆左转45度并前进"

#### 步骤4: 观察执行结果

**在CARLA仿真器中观察**:
- 车辆是否按照指令执行动作
- 动作是否流畅和准确

**在RoboOS日志中检查**:
```bash
# 查看Slaver日志（skill执行）
tail -f /tmp/carla_roboos_logs/terminal3_roboos_slaver.log

# 查看Master日志（任务分解）
tail -f /tmp/carla_roboos_logs/terminal2_roboos_master.log
```

---

### 方法3: 使用Python直接调用Skills（调试用）

**目的**: 直接测试skill函数，不通过UI

```python
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/dora/RoboOS/slaver/demo_robot_local')

from vehicle_carla.skills.control import VehicleController

# 初始化控制器
controller = VehicleController(host="192.168.1.1", port=23456)

# 测试1: 前进
print("Test 1: Move forward")
result = controller.move_forward(5.0)
print(f"Result: {result}\n")

# 测试2: 转向
print("Test 2: Turn right")
result = controller.turn_vehicle(30.0)
print(f"Result: {result}\n")

# 测试3: 停止
print("Test 3: Stop")
result = controller.stop_vehicle()
print(f"Result: {result}\n")
```

---

## 故障排查

### 问题1: UDP发送成功但车辆不动

**可能原因**:
- CARLA的dora.py agent未正确监听UDP端口
- 网络地址配置错误

**检查方法**:
```bash
# 检查UDP端口是否被监听
netstat -ulnp | grep 23456

# 检查leaderboard日志中的UDP接收信息
grep -i "udp\|control" /tmp/carla_roboos_logs/terminal2_leaderboard.log
```

### 问题2: RoboOS无法调用skills

**可能原因**:
- Slaver未正确加载vehicle_carla模块
- MCP服务器未启动

**检查方法**:
```bash
# 查看Slaver启动日志
head -50 /tmp/carla_roboos_logs/terminal3_roboos_slaver.log

# 检查是否有vehicle_carla相关错误
grep -i "vehicle_carla\|error" /tmp/carla_roboos_logs/terminal3_roboos_slaver.log
```

### 问题3: 网络连接超时

**可能原因**:
- IP地址配置错误
- 防火墙阻止UDP通信

**解决方法**:
```bash
# 检查网络配置
ip addr show

# 临时关闭防火墙测试（仅测试用）
sudo ufw disable

# 测试UDP连接
nc -u 192.168.1.1 23456
```

---

## 已注册的Skills列表

### 车辆控制类 (5个)
1. `set_vehicle_control(steer, throttle, brake)` - 设置车辆控制
2. `emergency_brake()` - 紧急制动
3. `stop_vehicle()` - 停止车辆
4. `move_forward(speed)` - 前进到指定速度
5. `turn_vehicle(angle)` - 转向到指定角度

### 传感器数据类 (3个)
6. `get_gnss_data()` - 获取GPS位置
7. `get_imu_data()` - 获取IMU数据
8. `get_vehicle_status()` - 获取车辆状态

### 仿真管理类 (3个)
9. `start_carla_simulation(scenario)` - 启动仿真
10. `stop_carla_simulation()` - 停止仿真
11. `get_simulation_status()` - 获取仿真状态

---

## 测试检查清单

- [ ] CARLA服务器正在运行
- [ ] Leaderboard正在运行
- [ ] 方法1: 直接UDP测试通过
- [ ] RoboOS四个组件全部启动
- [ ] 方法2: UI指令测试通过
- [ ] 车辆在CARLA中正确响应指令
- [ ] 日志中无错误信息

---

## 下一步

测试通过后，可以：
1. 开发更复杂的导航和路径规划功能
2. 集成传感器数据反馈
3. 实现自动驾驶任务序列
4. 添加性能监控和日志分析

---

**维护者**: SunSunSun689
**最后更新**: 2026-01-16
