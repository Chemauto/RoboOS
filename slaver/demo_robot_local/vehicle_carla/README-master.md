# RoboOS Vehicle-CARLA 模块化架构文档

**更新时间**: 2026-01-19

## 架构概述

RoboOS Vehicle-CARLA 采用模块化设计，通过适配器模式将车辆控制功能集成到 MCP (Model Context Protocol) 工具系统中。

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    用户UI界面 (Deploy)                           │
│  输入自然语言指令：\"让车辆前进5米每秒\"                            │
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
│  - 统一入口，注册所有模块                                        │
│  - 调用模块适配器                                                │
└────────────────────────────┬────────────────────────────────────┘
                             │ Python函数调用
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│           MCP工具适配层 (module/*.py)                            │
│  - vehicle_simulation.py: 仿真管理适配器                        │
│  - vehicle_sensor.py: 传感器数据适配器                          │
│  - vehicle_control.py: 车辆控制适配器                           │
│  - 包装为 @mcp.tool() 异步函数                                  │
│  - 返回 Tuple[str, Dict] 格式                                   │
└────────────────────────────┬────────────────────────────────────┘
                             │ 导入和实例化
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│        业务逻辑层 (vehicle_carla/skills/*.py)                    │
│  - SimulationManager: 仿真管理                                   │
│  - SensorReader: 传感器数据读取                                 │
│  - VehicleController: 车辆控制                                   │
│  - 实现具体业务逻辑                                              │
└────────────────────────────┬────────────────────────────────────┘
                             │ UDP/网络通信
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│              UDPClient (utils/udp_client.py)                     │
│  - 构造JSON控制消息                                              │
│  - 通过UDP发送到127.0.0.1:23456                                 │
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
                             │ CARLA API调用
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│                  CARLA仿真器                                     │
│  - 应用控制命令到车辆                                            │
│  - 更新车辆状态（位置、速度、转向）                              │
│  - 渲染仿真画面                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 模块化架构详解

### 1. 三层架构设计

#### 第一层：业务逻辑层 (`vehicle_carla/skills/`)

**职责**: 实现具体的车辆控制、传感器读取、仿真管理逻辑

**文件结构**:
```
vehicle_carla/skills/
├── __init__.py
├── simulation.py      # SimulationManager类 - 仿真管理
├── sensor.py          # SensorReader类 - 传感器数据读取
└── control.py         # VehicleController类 - 车辆控制
```

**特点**:
- 类基础设计，可独立使用
- 不依赖MCP框架
- 可在其他项目中复用

**示例** (`control.py`):
```python
class VehicleController:
    def __init__(self, host: str = "127.0.0.1", port: int = 23456):
        self.udp_client = UDPClient(host, port)

    def set_vehicle_control(self, steer: float, throttle: float, brake: float) -> str:
        # 参数验证和范围限制
        steer = max(-1.0, min(1.0, steer))
        throttle = max(0.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))

        # 发送UDP控制命令
        success = self.udp_client.send_control(steer, throttle, brake)
        return f"Control command sent: steer={steer}, throttle={throttle}, brake={brake}"
```

#### 第二层：MCP工具适配层 (`module/`)

**职责**: 将业务逻辑适配为MCP工具，遵循RoboOS模块化规范

**文件结构**:
```
module/
├── __init__.py
├── vehicle_simulation.py   # 仿真管理MCP适配器
├── vehicle_sensor.py        # 传感器数据MCP适配器
└── vehicle_control.py       # 车辆控制MCP适配器
```

**特点**:
- 使用 `@mcp.tool()` 装饰器
- 异步函数设计
- 返回 `Tuple[str, Dict]` 格式
- 日志输出到 `sys.stderr`
- 包含完整的文档字符串

**示例** (`module/vehicle_control.py`):
```python
from vehicle_carla.skills.control import VehicleController

_vehicle_controller = None

def get_vehicle_controller():
    global _vehicle_controller
    if _vehicle_controller is None:
        _vehicle_controller = VehicleController()
    return _vehicle_controller

def register_tools(mcp):
    @mcp.tool()
    async def set_vehicle_control(steer: float, throttle: float, brake: float) -> Tuple[str, Dict]:
        """Set vehicle control command with steering, throttle, and brake.

        Args:
            steer: Steering angle [-1.0, 1.0]
            throttle: Throttle [0.0, 1.0]
            brake: Brake [0.0, 1.0]

        Returns:
            A tuple containing the result message and control state.
        """
        print(f"[vehicle_control.set_vehicle_control] Setting control", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.set_vehicle_control(steer, throttle, brake)

        state_update = {
            "steer": steer,
            "throttle": throttle,
            "brake": brake
        }

        return result, state_update
```

#### 第三层：统一入口 (`skill.py`)

**职责**: 注册所有模块，启动MCP服务器

**关键代码**:
```python
from mcp.server.fastmcp import FastMCP

# 导入车辆CARLA模块
from module.vehicle_simulation import register_tools as register_vehicle_simulation_tools
from module.vehicle_sensor import register_tools as register_vehicle_sensor_tools
from module.vehicle_control import register_tools as register_vehicle_control_tools

mcp = FastMCP("robots")

def register_all_modules():
    # 注册车辆CARLA模块
    register_vehicle_simulation_tools(mcp)
    register_vehicle_sensor_tools(mcp)
    register_vehicle_control_tools(mcp)

if __name__ == "__main__":
    register_all_modules()
    mcp.run(transport="stdio")
```

### 2. 适配器模式关系

```
业务逻辑层                    MCP适配层                    统一入口
─────────────────────────────────────────────────────────────────
control.py                    vehicle_control.py           skill.py
├─ VehicleController          ├─ get_vehicle_controller()  ├─ register_all_modules()
│  ├─ set_vehicle_control()   │  └─ 单例管理                │  ├─ 导入所有模块
│  ├─ emergency_brake()       ├─ register_tools(mcp)       │  └─ 启动MCP服务器
│  ├─ stop_vehicle()          │  ├─ @mcp.tool()            │
│  ├─ move_forward()          │  │  set_vehicle_control()  │
│  └─ turn_vehicle()          │  ├─ @mcp.tool()            │
│                              │  │  emergency_brake()      │
sensor.py                     │  └─ ...                    │
├─ SensorReader               │                             │
│  ├─ get_gnss_data()         vehicle_sensor.py            │
│  ├─ get_imu_data()          ├─ get_sensor_reader()       │
│  └─ get_vehicle_status()    └─ register_tools(mcp)       │
│                                 ├─ @mcp.tool()            │
simulation.py                     │  get_gnss_data()        │
├─ SimulationManager              └─ ...                    │
   ├─ start_carla_simulation()                              │
   ├─ stop_carla_simulation()  vehicle_simulation.py        │
   └─ get_simulation_status()  ├─ get_sim_manager()         │
                                └─ register_tools(mcp)       │
                                   ├─ @mcp.tool()            │
                                   │  start_carla_simulation()
                                   └─ ...                    │
```

### 3. 已注册的MCP工具列表

#### 仿真管理工具 (3个)
- `start_carla_simulation(scenario: str)` - 启动CARLA仿真
- `stop_carla_simulation()` - 停止CARLA仿真
- `get_simulation_status()` - 获取仿真状态

#### 传感器数据工具 (3个)
- `get_gnss_data()` - 获取GPS位置数据
- `get_imu_data()` - 获取IMU数据（加速度、角速度、航向）
- `get_vehicle_status()` - 获取车辆综合状态

#### 车辆控制工具 (5个)
- `set_vehicle_control(steer, throttle, brake)` - 设置车辆控制命令
- `emergency_brake()` - 紧急刹车
- `stop_vehicle()` - 停止车辆
- `move_forward(speed)` - 前进到指定速度
- `turn_vehicle(angle)` - 转向到指定角度

**总计**: 11个MCP工具

## 通信协议详解

### UDP控制命令格式

**发送方**: RoboOS (UDPClient)
**接收方**: CARLA dora.py Agent
**地址**: 127.0.0.1:23456
**协议**: UDP
**格式**: JSON

```json
{
  "id": "control",
  "steer": 0.5,      // [-1.0, 1.0] 负值左转，正值右转
  "throttle": 0.3,   // [0.0, 1.0] 油门
  "brake": 0.0       // [0.0, 1.0] 制动
}
```

### UDP传感器数据格式

**发送方**: CARLA dora.py Agent
**接收方**: RoboOS (SensorReader)
**地址**: 192.168.1.101:12345
**协议**: UDP
**格式**: JSON

**GNSS数据**:
```json
{
  "id": "gnss",
  "x": -119.5,
  "y": 5505.3,
  "z": 370.0
}
```

**IMU数据**:
```json
{
  "id": "imu",
  "accelerometer": {"x": 0.1, "y": 0.0, "z": 9.8},
  "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0},
  "heading_deg": 90.0
}
```

## 完整调用流程示例

### 示例：用户指令 "让车辆前进5米每秒"

```
1. 用户UI输入
   └─> "让车辆前进5米每秒"

2. RoboOS Master (LLM分解)
   └─> 识别为 move_forward(speed=5.0) 工具调用

3. Redis任务队列
   └─> 发布任务到 slaver 订阅的频道

4. RoboOS Slaver
   └─> 接收任务，调用 MCP 服务器

5. FastMCP Server (skill.py)
   └─> 路由到 move_forward 工具

6. MCP适配层 (module/vehicle_control.py)
   └─> @mcp.tool() move_forward(speed=5.0)
       ├─> 获取 VehicleController 实例
       └─> 调用 controller.move_forward(5.0)

7. 业务逻辑层 (vehicle_carla/skills/control.py)
   └─> VehicleController.move_forward(5.0)
       ├─> 计算 throttle = 5.0 / 10.0 = 0.5
       └─> 调用 udp_client.send_control(0.0, 0.5, 0.0)

8. UDP通信层 (utils/udp_client.py)
   └─> 构造JSON: {"id": "control", "steer": 0.0, "throttle": 0.5, "brake": 0.0}
       └─> 发送到 127.0.0.1:23456

9. CARLA Agent (dora.py - receive_control_loop线程)
   └─> 接收UDP消息
       └─> 更新 self.control_command

10. CARLA Agent (dora.py - run_step方法)
    └─> 读取 control_command
        └─> 创建 VehicleControl(steer=0.0, throttle=0.5, brake=0.0)
            └─> 返回给 Leaderboard

11. CARLA仿真器
    └─> 应用控制命令
        └─> 车辆以5m/s速度前进
```

## 设计优势

### 1. 分离关注点
- **业务逻辑** 与 **MCP集成** 分离
- 修改控制逻辑不影响MCP接口
- 修改MCP接口不影响业务逻辑

### 2. 可复用性
- `vehicle_carla/skills/` 中的类可在其他项目中独立使用
- 不依赖RoboOS框架

### 3. 易维护性
- 模块化设计，职责清晰
- 每个文件功能单一
- 易于测试和调试

### 4. 可扩展性
- 添加新功能只需：
  1. 在 `vehicle_carla/skills/` 添加业务逻辑类
  2. 在 `module/` 创建MCP适配器
  3. 在 `skill.py` 注册模块

### 5. 类型安全
- 完整的类型注解
- 参数验证和范围限制
- 返回值格式统一

## 测试验证

### 1. 单元测试（业务逻辑层）
```python
# 测试 VehicleController
controller = VehicleController()
result = controller.move_forward(5.0)
assert "Moving forward at 5.00 m/s" in result
```

### 2. 集成测试（MCP工具层）
```bash
# 启动 skill.py
cd /home/dora/RoboOS/slaver/demo_robot_local
python skill.py
```

### 3. 端到端测试（完整系统）
```bash
# 启动RoboOS四个组件
cd /home/dora/RoboOS
./act-four-terminal.sh

# 通过UI发送指令
# "让车辆前进5米每秒"
```

## 网络配置

| 方向 | 协议 | 地址 | 端口 | 数据类型 | 用途 |
|------|------|------|------|---------|------|
| RoboOS → CARLA | UDP | 127.0.0.1 | 23456 | 控制命令 (JSON) | 车辆控制 |
| CARLA → RoboOS | UDP | 192.168.1.101 | 12345 | GNSS/IMU数据 (JSON) | 传感器数据 |

## 故障排查

### 问题1: 车辆不响应控制命令
**检查**:
1. CARLA Agent是否在监听UDP端口23456
2. UDP消息是否发送成功
3. 控制命令格式是否正确

**解决**:
```bash
# 测试UDP通信
cd /home/dora/RoboOS
python3 test_carla_communication.py
```

### 问题2: 传感器数据读取失败
**检查**:
1. CARLA Agent是否在发送传感器数据
2. UDP端口12345是否被占用
3. 网络地址配置是否正确

**解决**:
```python
# 检查传感器读取器配置
sensor_reader = SensorReader(host="192.168.1.101", port=12345)
```

### 问题3: MCP工具未注册
**检查**:
1. 模块导入是否成功
2. register_tools() 是否被调用
3. 查看启动日志

**解决**:
```bash
# 查看启动日志
cd /home/dora/RoboOS/slaver/demo_robot_local
python skill.py 2>&1 | grep "已注册"
```

## 开发指南

### 添加新的车辆控制功能

**步骤1**: 在业务逻辑层添加方法
```python
# vehicle_carla/skills/control.py
class VehicleController:
    def reverse(self, speed: float) -> str:
        throttle = min(speed / 10.0, 1.0)
        success = self.udp_client.send_control(0.0, -throttle, 0.0)
        return f"Reversing at {speed} m/s"
```

**步骤2**: 在MCP适配层添加工具
```python
# module/vehicle_control.py
def register_tools(mcp):
    @mcp.tool()
    async def reverse(speed: float) -> Tuple[str, Dict]:
        """Reverse the vehicle at specified speed."""
        controller = get_vehicle_controller()
        result = controller.reverse(speed)
        return result, {"speed": speed, "direction": "reverse"}
```

**步骤3**: 重启服务
```bash
# 重启 slaver 服务
cd /home/dora/RoboOS
# 重新运行 act-four-terminal.sh
```

## MCP工具测试指南

### 已注册的11个MCP工具

#### 1. 仿真管理工具 (3个)

| 工具名称 | 测试指令示例 | 关键词 |
|---------|------------|--------|
| `start_carla_simulation` | "启动CARLA仿真"<br>"开始仿真,使用默认场景"<br>"启动仿真,场景名称为urban" | 启动, 开始, 仿真, simulation, start, scenario |
| `stop_carla_simulation` | "停止CARLA仿真"<br>"结束仿真"<br>"关闭仿真环境" | 停止, 结束, 关闭, 仿真, simulation, stop |
| `get_simulation_status` | "查看仿真状态"<br>"仿真是否在运行"<br>"获取仿真状态信息" | 查看, 获取, 状态, 仿真, simulation, status, running |

#### 2. 传感器数据工具 (3个)

| 工具名称 | 测试指令示例 | 关键词 |
|---------|------------|--------|
| `get_gnss_data` | "获取GPS位置数据"<br>"查看当前位置"<br>"读取GNSS数据" | GPS, GNSS, 位置, 坐标, position, location |
| `get_imu_data` | "获取IMU数据"<br>"查看加速度和角速度"<br>"读取航向角" | IMU, 加速度, 角速度, 航向, heading, accelerometer, gyroscope |
| `get_vehicle_status` | "获取车辆状态"<br>"查看车辆当前状态"<br>"车辆位置和航向" | 车辆, 状态, vehicle, status, 综合 |

#### 3. 车辆控制工具 (5个)

| 工具名称 | 测试指令示例 | 关键词 |
|---------|------------|--------|
| `set_vehicle_control` | "设置转向0.5,油门0.3,刹车0"<br>"控制车辆:转向角度0.2,油门0.5,制动0"<br>"设置车辆控制参数" | 设置, 控制, 转向, 油门, 刹车, steer, throttle, brake, control |
| `emergency_brake` | "紧急刹车"<br>"立即制动"<br>"急刹车" | 紧急, 急刹, emergency, brake, 立即 |
| `stop_vehicle` | "停止车辆"<br>"让车停下来"<br>"停车" | 停止, 停车, stop, vehicle, 停下 |
| `move_forward` | "让车辆前进,速度5米每秒"<br>"前进到10m/s"<br>"以3米每秒的速度前进" | 前进, 向前, forward, move, 速度, speed, 米每秒, m/s |
| `turn_vehicle` | "让车辆右转30度"<br>"向左转20度"<br>"转向角度-15度" | 转向, 转弯, turn, 左转, 右转, 角度, angle, 度 |

### 关键词分类汇总

#### 仿真管理关键词
```
中文: 启动, 开始, 停止, 结束, 关闭, 查看, 状态, 仿真
英文: start, stop, simulation, status, scenario
```

#### 传感器数据关键词
```
中文: 获取, 查看, 读取, 位置, 坐标, 加速度, 角速度, 航向, 车辆状态
英文: GPS, GNSS, IMU, get, read, position, location, accelerometer, gyroscope, heading, vehicle, status
```

#### 车辆控制关键词
```
中文: 设置, 控制, 转向, 油门, 刹车, 制动, 紧急, 急刹, 停止, 停车, 前进, 向前, 转弯, 左转, 右转, 速度, 角度
英文: set, control, steer, throttle, brake, emergency, stop, move, forward, turn, speed, angle
```

### 测试流程建议

#### 阶段1: 基础功能测试
```
1. "启动CARLA仿真"
2. "查看仿真状态"
3. "获取车辆状态"
4. "让车辆前进,速度3米每秒"
5. "停止车辆"
6. "停止仿真"
```

#### 阶段2: 传感器数据测试
```
1. "获取GPS位置数据"
2. "获取IMU数据"
3. "查看车辆当前状态"
```

#### 阶段3: 高级控制测试
```
1. "让车辆右转20度"
2. "让车辆左转30度"
3. "设置转向0.3,油门0.5,刹车0"
4. "紧急刹车"
```

#### 阶段4: 组合任务测试
```
1. "让车辆前进5米每秒,然后右转15度"
2. "获取当前位置,然后前进10米每秒"
3. "查看车辆状态,如果速度过快就停车"
```

### 测试环境准备

**前置条件:**
1. CARLA服务器已启动
2. Leaderboard评估框架已运行
3. RoboOS四个组件已启动 (Master, Slaver, Deploy, Redis)

**启动命令:**
```bash
# 启动RoboOS系统
cd /home/dora/RoboOS
./act-four-terminal.sh
```

**测试方式:**
- 通过Deploy UI界面输入自然语言指令
- Master的LLM会识别关键词并调用相应的MCP工具
- 观察车辆在CARLA仿真器中的响应

### 预期结果

| 测试指令 | 预期结果 | 验证方法 |
|---------|---------|---------|
| "启动CARLA仿真" | 仿真状态变为运行中 | 查看日志输出 |
| "获取GPS位置数据" | 返回x, y, z坐标 | 检查返回的坐标值 |
| "让车辆前进,速度5米每秒" | 车辆开始前进 | 观察CARLA画面中车辆移动 |
| "让车辆右转30度" | 车辆向右转向 | 观察车辆转向角度 |
| "停止车辆" | 车辆停止移动 | 车辆速度降为0 |
| "紧急刹车" | 车辆立即制动 | 车辆快速停止 |

### 调试技巧

**查看MCP工具调用日志:**
```bash
# 查看slaver日志
tail -f slaver/.log/agent.log

# 查看skill.py输出
# 在启动时会显示所有注册的工具
```

**验证工具是否注册成功:**
```bash
cd /home/dora/RoboOS/slaver/demo_robot_local
python skill.py 2>&1 | grep "已注册"

# 应该看到:
# [vehicle_simulation.py] CARLA仿真管理模块已注册
# [vehicle_sensor.py] 车辆传感器模块已注册
# [vehicle_control.py] 车辆控制模块已注册
```

**测试UDP通信:**
```bash
cd /home/dora/RoboOS
python3 test_carla_communication.py
```

## 参考文档

- [RoboOS模块化开发指南](../README_MOUDLES.md)
- [FastMCP文档](https://github.com/jlowin/fastmcp)
- [CARLA文档](https://carla.readthedocs.io/)

---

**最后更新**: 2026-01-19
**维护者**: RoboOS开发团队
