# RoboOS与CARLA通信架构详解

**更新时间**: 2026-01-16

## 完整通信流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    用户UI界面 (Deploy)                           │
│  输入自然语言指令："让车辆前进5米每秒"                            │
└────────────────────────────┬────────────────────────────────────┘
                             │ HTTP请求
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                    RoboOS Master                                 │
│  - 接收用户指令                                                   │
│  - LLM分解任务为具体skills调用                                   │
│  - 发布任务到Redis                                               │
└────────────────────────────┬────────────────────────────────────┘
                             │ Redis Pub/Sub
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                    RoboOS Slaver                                 │
│  - 订阅Redis任务队列                                             │
│  - 加载本地skill模块 (demo_robot_local)                         │
│  - 调用MCP服务器                                                 │
└────────────────────────────┬────────────────────────────────────┘
                             │ MCP Protocol (stdio)
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│              FastMCP Server (skill.py)                           │
│  - 注册11个vehicle_carla skills                                  │
│  - 接收skill调用请求                                             │
│  - 调用VehicleController                                         │
└────────────────────────────┬────────────────────────────────────┘
                             │ Python函数调用
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│         VehicleController (control.py)                           │
│  - 参数验证和范围限制                                            │
│  - 调用UDPClient发送控制命令                                     │
└────────────────────────────┬────────────────────────────────────┘
                             │ Python函数调用
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│              UDPClient (udp_client.py)                           │
│  - 构造JSON控制消息                                              │
│  - 通过UDP发送到192.168.1.1:23456                               │
└────────────────────────────┬────────────────────────────────────┘
                             │ UDP网络传输
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│         CARLA dora.py Agent (receive_control_loop线程)           │
│  - 监听UDP端口23456                                              │
│  - 接收JSON控制消息                                              │
│  - 更新self.control_command                                      │
└────────────────────────────┬────────────────────────────────────┘
                             │ 线程共享变量
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│         CARLA dora.py Agent (run_step方法)                       │
│  - 每帧读取control_command                                       │
│  - 创建VehicleControl对象                                        │
│  - 返回给Leaderboard框架                                         │
└────────────────────────────┬────────────────────────────────────┘
                             │ 函数返回值
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│              Leaderboard评估框架                                 │
│  - 接收VehicleControl对象                                        │
│  - 传递给CARLA仿真器                                             │
└────────────────────────────┬────────────────────────────────────┘
                             │ CARLA API调用
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                  CARLA仿真器                                     │
│  - 应用控制命令到车辆                                            │
│  - 更新车辆状态（位置、速度、转向）                              │
│  - 渲染仿真画面                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 关键组件详解

### 1. RoboOS Slaver配置 (`slaver/config.yaml`)
```yaml
robot:
  call_type: local
  path: "demo_robot_local"
  name: demo_robot
```
- 指定使用本地skill模块
- 路径指向 `demo_robot_local` 目录

### 2. FastMCP Skill注册 (`skill.py`)
```python
from mcp.server.fastmcp import FastMCP
from vehicle_carla.skills.control import VehicleController

mcp = FastMCP("robots")
controller = VehicleController()

@mcp.tool()
async def move_forward(speed: float) -> str:
    return controller.move_forward(speed)
```
- 使用FastMCP框架注册skills
- 每个skill对应一个异步函数
- 调用VehicleController的方法

### 3. UDP控制命令格式 (`udp_client.py`)
```json
{
  "id": "control",
  "steer": 0.5,      // [-1.0, 1.0] 负值左转，正值右转
  "throttle": 0.3,   // [0.0, 1.0] 油门
  "brake": 0.0       // [0.0, 1.0] 制动
}
```

### 4. CARLA Agent接收 (`dora.py`)
```python
def receive_control_loop(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.1.1", 23456))
    while True:
        data, _ = sock.recvfrom(1024)
        msg = json.loads(data.decode('utf-8'))
        if msg.get("id") == "control":
            with self.control_lock:
                self.control_command["steer"] = float(msg.get("steer", 0.0))
                self.control_command["throttle"] = float(msg.get("throttle", 0.0))
                self.control_command["brake"] = float(msg.get("brake", 0.0))
```
- 后台线程持续监听UDP端口
- 使用线程锁保护共享变量
- 默认值为制动状态 (brake=1.0)

### 5. 控制命令应用 (`dora.py`)
```python
def run_step(self, input_data, timestamp):
    # ... 处理传感器数据 ...

    control = VehicleControl()
    with self.control_lock:
        control.steer = self.control_command["steer"]
        control.throttle = self.control_command["throttle"]
        control.brake = self.control_command["brake"]
    return control
```
- 每帧调用一次（20-30 FPS）
- 读取最新的控制命令
- 返回给CARLA执行

## 网络配置

| 方向 | 协议 | 地址 | 端口 | 数据类型 |
|------|------|------|------|---------|
| RoboOS → CARLA | UDP | 192.168.1.1 | 23456 | 控制命令 (JSON) |
| CARLA → 外部 | UDP | 192.168.1.101 | 12345 | GNSS/IMU数据 (JSON) |
| CARLA → 外部 | TCP | 192.168.1.101 | 5005 | LiDAR点云 (二进制) |

## 通信特点

1. **异步通信**: UDP无连接，低延迟
2. **线程安全**: 使用锁保护共享变量
3. **容错设计**: 默认制动状态，超时处理
4. **解耦架构**: RoboOS和CARLA通过UDP松耦合
5. **实时性**: 每帧更新控制命令（20-30Hz）

## 测试验证

已验证通信链路正常：
```bash
cd /home/dora/RoboOS
python3 test_carla_communication.py
```

结果：
- ✅ UDP通信正常
- ✅ JSON格式正确
- ✅ CARLA agent正在监听
- ✅ 控制命令被接收和应用

---

# Vehicle-CARLA 场景选择指南

## 测试场景对比

项目提供三个测试场景文件（位于 `Vehicle/CARLA_Leaderboard_20/leaderboard/data/`）：

| 场景文件 | 起点坐标 | 特点 | 推荐度 |
|---------|---------|------|--------|
| **Pure_Tracking_Scenario.xml** | (-119.5, 5505.3, 370) | 纯路径跟踪，起点有1100米长直道 | ⭐⭐⭐⭐⭐ |
| AEB_Scenario.xml | (-710.9, 3650.8, 365) | 自动紧急制动，多突发情况 | ⭐⭐⭐ |
| Obstacle_Avoidance_Scenario.xml | (-817.1, 6083.4, 368.5) | 避障场景，多障碍物 | ⭐⭐⭐ |

## 推荐方案

### ✅ 强烈推荐：Pure_Tracking_Scenario.xml

**适用场景**：UI界面控制测试、基础通信验证、RoboOS框架集成测试

**优势**：
- 起点到第一路点是1100米长直道，适合测试基础控制
- 视野开阔，便于观察车辆运动
- 场景侧重路径跟踪，不是紧急情况处理
- 已验证UDP通信成功

**测试流程**：
```
用户UI输入 → RoboOS Master分解 → Slaver调用skills → UDP控制 → CARLA响应
```

## 测试指令示例

### 阶段1：基础控制（在起点直道测试）
- "让车辆前进，速度3米每秒"
- "让车辆停止"
- "让车辆右转20度"
- "让车辆左转20度"
- "紧急制动"

### 阶段2：状态查询
- "获取车辆当前位置"
- "获取车辆状态"
- "获取GPS数据"

### 阶段3：组合任务
- "让车辆前进50米然后停止"
- "让车辆转向并前进"

## 场景切换方法

修改 `Vehicle/CARLA_Leaderboard_20/leaderboard/test_run.sh` 第24行：

```bash
# 当前使用（推荐）
export ROUTES=${LEADERBOARD_ROOT}/data/Pure_Tracking_Scenario.xml

# 切换到AEB场景（测试紧急制动时使用）
# export ROUTES=${LEADERBOARD_ROOT}/data/AEB_Scenario.xml

# 切换到避障场景（测试避障算法时使用）
# export ROUTES=${LEADERBOARD_ROOT}/data/Obstacle_Avoidance_Scenario.xml
```

## 其他场景使用时机

**AEB_Scenario.xml**：
- 重点测试 `emergency_brake()` skill
- 验证紧急情况响应能力

**Obstacle_Avoidance_Scenario.xml**：
- 测试避障算法
- 验证传感器数据处理和决策

## 快速测试

### 1. 直接UDP通信测试
```bash
cd /home/dora/RoboOS
python3 test_carla_communication.py
```

### 2. 完整RoboOS系统测试
```bash
# 启动RoboOS四个组件
./act-four-terminal.sh

# 通过UI发送指令测试
```

## 注意事项

- 确保CARLA服务器和Leaderboard已启动
- UDP通信地址：192.168.1.1:23456
- 所有场景均在Town12地图运行
- 建议先用Pure_Tracking场景验证基础功能，再切换到其他场景测试特定功能
