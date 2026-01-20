● 以下是可以在UI界面输入的测试指令,覆盖所有20个已注册工具:

  测试指令清单 (按模块分类)

  一、车辆控制模块 (12个工具)

  基础控制

  1. 设置车辆控制,方向盘角度0.5,油门0.3,刹车0
  2. 紧急刹车
  3. 停止车辆

  前进控制

  4. 以5米每秒的速度前进
  5. 向前移动3米,速度2米每秒

  转向控制

  6. 向右转30度
  7. 调用turn_vehicle_angle函数,设置目标角度为45度,使用默认速度

  倒车控制 (新增)

  8. 以2米每秒的速度倒车
  9. 倒车3米,速度1.5米每秒

  快速查询 (新增)

  10. 获取当前车辆朝向
  11. 获取当前车辆位置

  传感器数据 (新增)

  12. 获取IMU原始传感器数据

  二、传感器模块 (5个工具)

  传感器读取

  13. 获取GPS位置数据
  14. 获取IMU数据
  15. 获取车辆状态

  原始数据 (新增)

  16. 获取GNSS原始传感器数据,超时2秒

  三、仿真管理模块 (3个工具)

  仿真控制

  17. 启动CARLA仿真,场景为Town04
  18. 获取仿真状态
  19. 停止CARLA仿真

  四、资源清理 (新增)

  20. 关闭所有车辆连接

  推荐测试顺序

  阶段1: 基础功能测试 (5分钟)

  1. 获取车辆状态
  2. 停止车辆
  3. 向前移动2米,速度1米每秒
  4. 停止车辆
  5. 获取当前位置

  阶段2: 转向测试 (3分钟)

  6. 获取当前朝向
  7. 向右转20度
  8. 停止车辆
  9. 获取当前朝向

  阶段3: 倒车测试 (3分钟)

  10. 倒车2米,速度1米每秒
  11. 停止车辆
  12. 获取当前位置

  阶段4: 传感器测试 (2分钟)

  13. 获取GPS位置数据
  14. 获取IMU数据
  15. 获取IMU原始传感器数据

  阶段5: 综合测试 (5分钟)

  16. 向前移动3米,速度2米每秒
  17. 向右转45度
  18. 向前移动2米,速度1.5米每秒
  19. 倒车1米,速度1米每秒
  20. 停止车辆

  快速测试脚本 (一键测试)

  如果想快速验证所有功能,可以依次输入:

  测试1: 停止车辆
  测试2: 获取车辆状态
  测试3: 向前移动1米
  测试4: 获取当前位置
  测试5: 向右转15度
  测试6: 获取当前朝向
  测试7: 倒车1米
  测试8: 停止车辆
  测试9: 获取GPS位置数据
  测试10: 紧急刹车

  注意事项

  1. 测试前准备:
    - 确保 CARLA 仿真已启动 (python3 simple_vehicle_control.py)
    - 确保 slaver 服务已重启(加载新工具)
  2. 测试间隔:
    - 每个指令之间等待2-3秒,让车辆完成动作
    - 观察终端输出确认指令执行
  3. 异常处理:
    - 如果出现 "Turn timeout",说明转向时间不够,这是正常的
    - 如果出现 "Position: N/A",检查传感器数据是否正常接收
  4. 验证成功标志:
    - 终端显示 [vehicle_control.xxx] 日志
    - 返回结果包含 "Successfully" 或具体数据
    - CARLA画面中车辆有相应动作

  预期输出示例

  输入: "向前移动2米,速度1米每秒"

  预期输出:
  [vehicle_control.move_forward_distance] Moving 2.0m at 1.0m/s
  [vehicle_control] Moving 2.00m at 1.00m/s from (100.5, 200.3)
  [vehicle_control] Reached target: traveled 2.00m
  Observations: Successfully moved 2.00m




❯ 这几个通信方式分为几类?还有哪些通信方式? 

● 通信方式分类

  当前系统使用的通信方式分类

  1. 基于协议层分类
  ┌────────────┬──────────────────────┬──────────────────────────────────┐
  │    类别    │       通信方式       │             使用场景             │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │ 应用层协议 │ HTTP/WebSocket       │ UI → Master                      │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │            │ OpenAI API (HTTPS)   │ Slaver → LLM                     │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │ 传输层协议 │ UDP                  │ Controller ↔ CARLA               │
  │            │                      │ (控制+传感器)                    │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │ 进程间通信 │ MCP Session          │ Slaver → Tool Executor           │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │ 内存数据库 │ Redis (Collaborator) │ Master ↔ Slaver (状态共享)       │
  ├────────────┼──────────────────────┼──────────────────────────────────┤
  │ 库内调用   │ CARLA Python API     │ Python ↔ CARLA Simulator         │
  └────────────┴──────────────────────┴──────────────────────────────────┘
  2. 基于通信模式分类
  ┌───────────┬───────────────────────────────┬────────────────────────┐
  │   模式    │           通信方式            │          特点          │
  ├───────────┼───────────────────────────────┼────────────────────────┤
  │ 请求-响应 │ HTTP, OpenAI API, MCP Session │ 同步等待响应           │
  ├───────────┼───────────────────────────────┼────────────────────────┤
  │ 发布-订阅 │ Redis Collaborator            │ 异步状态共享           │
  ├───────────┼───────────────────────────────┼────────────────────────┤
  │ 数据报    │ UDP                           │ 无连接,低延迟,可能丢包 │
  ├───────────┼───────────────────────────────┼────────────────────────┤
  │ 回调机制  │ CARLA Sensor Callbacks        │ 事件驱动               │
  └───────────┴───────────────────────────────┴────────────────────────┘
  3. 基于可靠性分类
  ┌────────────┬─────────────────────────────┬─────────────────────────┐
  │   可靠性   │          通信方式           │          说明           │
  ├────────────┼─────────────────────────────┼─────────────────────────┤
  │ 可靠传输   │ HTTP, WebSocket, Redis, MCP │ 保证消息送达            │
  ├────────────┼─────────────────────────────┼─────────────────────────┤
  │ 不可靠传输 │ UDP                         │ 不保证送达,适合实时控制 │
  └────────────┴─────────────────────────────┴─────────────────────────┘
  其他常见通信方式(当前系统未使用)

● 1. 机器人领域常用
  ┌──────────────┬─────────────────┬───────────────────────┐
  │   通信方式   │      特点       │       适用场景        │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ ROS Topics   │ 发布-订阅,异步  │ 机器人传感器数据流    │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ ROS Services │ 请求-响应,同步  │ 机器人功能调用        │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ ROS Actions  │ 长时任务,带反馈 │ 导航、抓取等复杂任务  │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ DDS          │ 实时数据分发    │ ROS2底层,工业级实时性 │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ MQTT         │ 轻量级发布-订阅 │ IoT设备,低带宽场景    │
  ├──────────────┼─────────────────┼───────────────────────┤
  │ gRPC         │ 高性能RPC       │ 微服务间通信          │
  └──────────────┴─────────────────┴───────────────────────┘
  2. 分布式系统常用
  ┌──────────┬────────────────┬───────────────────┐
  │ 通信方式 │      特点      │     适用场景      │
  ├──────────┼────────────────┼───────────────────┤
  │ Kafka    │ 高吞吐消息队列 │ 日志收集,事件流   │
  ├──────────┼────────────────┼───────────────────┤
  │ RabbitMQ │ 可靠消息队列   │ 任务队列,异步处理 │
  ├──────────┼────────────────┼───────────────────┤
  │ ZeroMQ   │ 高性能消息库   │ 低延迟分布式通信  │
  ├──────────┼────────────────┼───────────────────┤
  │ etcd     │ 分布式键值存储 │ 配置管理,服务发现 │
  ├──────────┼────────────────┼───────────────────┤
  │ Consul   │ 服务网格       │ 服务注册与发现    │
  └──────────┴────────────────┴───────────────────┘
  3. 实时通信
  ┌────────────────────┬─────────────────┬──────────────────────┐
  │      通信方式      │      特点       │       适用场景       │
  ├────────────────────┼─────────────────┼──────────────────────┤
  │ TCP                │ 可靠流式传输    │ 需要保证顺序的数据流 │
  ├────────────────────┼─────────────────┼──────────────────────┤
  │ UDP Multicast      │ 一对多广播      │ 多机器人协同         │
  ├────────────────────┼─────────────────┼──────────────────────┤
  │ Shared Memory      │ 零拷贝,极低延迟 │ 同机器进程间高速通信 │
  ├────────────────────┼─────────────────┼──────────────────────┤
  │ Unix Domain Socket │ 本地进程通信    │ 同机器高性能IPC      │
  └────────────────────┴─────────────────┴──────────────────────┘
  4. 硬件接口
  ┌───────────────┬────────────┬───────────────┐
  │   通信方式    │    特点    │   适用场景    │
  ├───────────────┼────────────┼───────────────┤
  │ CAN Bus       │ 汽车总线   │ 车载ECU通信   │
  ├───────────────┼────────────┼───────────────┤
  │ Serial (UART) │ 串口通信   │ 单片机,传感器 │
  ├───────────────┼────────────┼───────────────┤
  │ I2C/SPI       │ 芯片间通信 │ 嵌入式传感器  │
  ├───────────────┼────────────┼───────────────┤
  │ Ethernet/IP   │ 工业以太网 │ 工业自动化    │
  └───────────────┴────────────┴───────────────┘
  当前系统可优化的通信方式

  基于RoboOS架构,以下是可能的改进方向:

  1. UDP → TCP (可选): 如果需要保证控制指令可靠送达
  2. Redis → ROS Topics: 如果扩展到多机器人协同
  3. HTTP → gRPC: Master-Slaver通信可改用gRPC提升性能
  4. 添加 WebSocket: 实时推送车辆状态到UI界面
  5. 添加 Shared Memory: CARLA与RoboOS同机时可用共享内存替代UDP

  当前系统的通信方式选择已经比较合理:
  - UDP: 适合实时控制(低延迟优先)
  - Redis: 适合状态共享(多节点访问)
  - MCP: 适合工具调用(标准化接口)


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


# 从UI输入到执行skill.py功能的完整流程

## 流程图

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Deploy UI (用户输入)                                      │
│    用户输入: "把苹果从桌子拿到厨房"                           │
└────────────────────┬────────────────────────────────────────┘
                     │ HTTP POST
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. Master (大脑 - 任务分解)                                  │
│    - 接收任务                                                │
│    - 使用LLM分解为子任务:                                     │
│      ① Navigate to table                                    │
│      ② Grasp apple                                          │
│      ③ Navigate to kitchen                                  │
│      ④ Place apple                                          │
└────────────────────┬────────────────────────────────────────┘
                     │ Redis Pub/Sub
                     │ Channel: "roboos_to_demo_robot"
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Slaver (小脑 - 任务接收)                                  │
│    run.py:256-258 监听Redis通道                              │
│    handle_task() 接收子任务: "Grasp apple"                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Tool Matcher (工具匹配)                                   │
│    run.py:123-133                                            │
│    - 使用语义匹配找到相关工具                                 │
│    - 输入: "Grasp apple"                                     │
│    - 输出: [("grasp_object", 0.92), ...]                     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. ToolCallingAgent (LLM决策)                                │
│    slaver_agent.py                                           │
│    - 使用LLM分析任务和可用工具                                │
│    - 决定调用: grasp_object(object="apple")                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. MCP Protocol (工具调用)                                   │
│    run.py:143 session.call_tool()                           │
│    - 通过MCP协议调用skill.py中的函数                          │
│    - 调用: grasp_object(object="apple")                      │
└────────────────────┬────────────────────────────────────────┘
                     │ stdio/HTTP
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. skill.py (功能执行)                                       │
│    @mcp.tool()                                               │
│    async def grasp_object(object: str):                      │
│        # 你的实现代码                                         │
│        result = f"{object} has been grasped"                 │
│        return result, {"grasped": object}                    │
└────────────────────┬────────────────────────────────────────┘
                     │ 返回结果
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 8. 结果返回                                                  │
│    Slaver → Redis → Master → Deploy UI                      │
│    显示: "apple has been successfully grasped"               │
└─────────────────────────────────────────────────────────────┘
```

## 关键代码位置

### 1. Slaver启动时连接MCP并获取工具列表 (run.py:184-229)

```python
# 连接到skill.py
server_params = StdioServerParameters(
    command="python",
    args=["demo_robot_local/skill.py"]  # 这里指定skill.py路径
)
session = ClientSession(...)
await session.initialize()

# 获取skill.py中注册的所有工具
response = await session.list_tools()
self.tools = [
    {
        "function": {
            "name": tool.name,           # 从@mcp.tool()获取
            "description": tool.description,
        }
    }
    for tool in response.tools
]
```

### 2. 接收任务后进行工具匹配 (run.py:123-133)

```python
task = "Grasp apple"
# 语义匹配找到最相关的工具
matched_tools = self.tool_matcher.match_tools(task)
# 结果: [("grasp_object", 0.92), ("navigate_to_target", 0.45)]

# 过滤出匹配的工具
filtered_tools = [tool for tool in self.tools
                 if tool["function"]["name"] in matched_tool_names]
```

### 3. LLM决策并调用工具 (run.py:135-146)

```python
agent = ToolCallingAgent(
    tools=filtered_tools,
    tool_executor=self.session.call_tool  # MCP调用接口
)

# LLM分析后决定调用哪个工具
result = await agent.run("Grasp apple")
# LLM输出: {"name": "grasp_object", "arguments": {"object": "apple"}}

# 通过MCP调用skill.py中的函数
await self.session.call_tool(
    name="grasp_object",
    arguments={"object": "apple"}
)
```

## 核心机制

1. **工具发现**: Slaver启动时通过`session.list_tools()`自动发现skill.py中所有`@mcp.tool()`装饰的函数

2. **语义匹配**: 使用sentence-transformers计算任务描述和工具描述的相似度，找到最相关的工具

3. **LLM决策**: 将匹配的工具列表和任务发给LLM，LLM决定调用哪个工具及参数

4. **MCP调用**: 通过`session.call_tool(name, arguments)`调用skill.py中对应的函数

## 总结

**你在skill.py中用`@mcp.tool()`注册的函数，会被自动发现并添加到工具列表中。当用户输入任务时，系统通过语义匹配+LLM决策找到最合适的工具，然后通过MCP协议调用你的实现代码。**

---

# RoboOS 项目分析

# RoboOS 代码分析

本文档旨在解析 RoboOS 项目的核心代码结构，重点介绍 `deploy`、`master` 和 `slaver` 三个关键组件的入口文件（`run.py`），以帮助理解整个系统的工作流程。

## 1. 整体架构

根据您的描述和代码分析，RoboOS 遵循一个经典且强大的“大脑-小脑-用户界面”分布式架构：

-   **`deploy` (用户界面 & 总控)**: 作为一个 Web 服务，为用户提供图形化操作界面。它是整个系统的总入口，负责**配置、校验、并启动** Master 和 Slaver 服务，以及底层的 AI 模型（RoboBrain）。
-   **`master` (大脑 & 任务规划器)**: 系统的决策中心。它接收来自 `deploy` UI 的高级任务指令，利用大语言模型（如 RoboBrain）的规划能力，将宏观任务**分解成一系列具体的、可执行的子任务**。
-   **`slaver` (小脑 & 任务执行器)**: 系统的执行单元，代表一个具体的机器人。它连接并控制机器人硬件或仿真环境，向 Master 注册自己拥有的**技能（Tools）**，接收并执行 Master 分配的子任务，并将执行结果反馈给 Master。

数据流向： `Deploy UI` -> `Master` -> `Slaver` -> `Robot`

## 2. 核心文件代码思路

### 2.1 `deploy/run.py` - UI & 服务控制器

该文件是整个系统的**用户界面**和**服务启动控制器**，基于 **Flask** Web 框架构建。

**核心功能:**

1.  **Web UI 服务**:
    -   通过 `@app.route("/")`, `@app.route("/deploy")` 等路由，向用户浏览器提供前端页面（`index.html`, `deploy.html` 等）。用户可以通过这个 UI 查看系统状态、配置并启动任务。

2.  **配置管理**:
    -   `/config`: 提供 API 读取 `master` 和 `slaver` 的 `config.yaml` 配置文件内容，并将其展示在前端。
    -   `/saveconfig`: 提供 API 接收前端修改后的配置数据，并将其保存回对应的 `config.yaml` 文件中。

3.  **环境校验**:
    -   `/api/validate-config`: 这是启动前的关键**预检**步骤。它会检查：
        -   端口（如 5000, 4567）是否被占用。
        -   `master` 和 `slaver` 的配置文件是否存在。
        -   两者配置中的通信中间件（Redis）信息是否一致且能成功连接。
        -   如果 Slaver 配置为远程模式，会检查远程机器人（MCP）服务是否可访问。

4.  **服务启动与管理**:
    -   通过 `subprocess.Popen` 在后台启动和管理核心服务，实现了与主 UI 进程的解耦。
    -   `/api/start-inference`: 启动底层的 vLLM 模型推理服务（即 RoboBrain）。
    -   `/api/start-master`: 启动 Master（大脑）服务。
    -   `/api/start-slaver`: 启动 Slaver（小脑）服务。
    -   每个启动请求都包含必要的 conda 环境名和配置文件路径，确保服务在正确的环境中运行。

### 2.2 `master/run.py` - 大脑 & 任务规划器

该文件是系统的**决策和任务规划中心**，同样基于 **Flask** 构建，主要用于提供 API 服务。

**核心功能:**

1.  **初始化核心 Agent**:
    -   脚本启动时，会创建一个 `GlobalAgent` 实例。这个 Agent 是 Master 的核心，封装了与大语言模型交互进行任务规划的主要逻辑。

2.  **接收和发布任务**:
    -   `/publish_task`: 这是最核心的 API 接口。它接收来自 `deploy` UI 的高级、模糊的任务指令（例如：“把桌上的苹果拿到冰箱里”）。
    -   接收到任务后，它会调用 `master_agent.publish_global_task()` 方法。该方法内部会与 RoboBrain LLM 通信，将这个高级任务**分解为一系列有序的、具体的子任务**（例如：`["找到桌子", "在桌子上寻找苹果", "抓取苹果", "找到冰箱", "打开冰箱门", "放入苹果", "关闭冰箱门"]`）。

3.  **状态监控**:
    -   `/robot_status`: 通过通信中间件（`collaborator`）查询当前所有已注册的 Slaver 机器人的状态，并返回给前端展示。
    -   `/system_status`: 提供 Master 服务所在机器的 CPU 和内存负载信息。

4.  **通信**:
    -   通过 `flask_socketio` 实现与前端的实时通信，可以主动推送更新（例如，任务分解的实时进度）。
    -   通过 `Collaborator` 对象（一个基于 Redis 的通信层）将分解后的子任务发布到指定的频道，等待 Slaver 订阅和执行。

### 2.3 `slaver/run.py` - 小脑 & 任务执行器

该文件是**机器人的直接控制和执行代理**，不包含 Web 服务，是一个基于 `asyncio` 的异步应用，用于高效处理 I/O 密集型任务（如网络通信和等待机器人动作完成）。

**核心功能:**

1.  **初始化 `RobotManager`**:
    -   `RobotManager` 类是 Slaver 的核心，负责管理与机器人的连接、与 Master 的通信以及任务的执行循环。
    -   在初始化时，它会加载配置文件、创建与 Master 通信的 `Collaborator` 实例，并根据配置初始化一个 LLM 客户端（用于执行任务时的决策）和一个 `ToolMatcher`（用于智能匹配任务和技能）。

2.  **连接和注册**:
    -   `connect_to_robot()`: 通过 `mcp` 协议库连接到机器人。支持两种模式：
        -   `local`: 直接在本地以子进程方式运行一个 `skill.py` 脚本。
        -   `remote`: 通过 HTTP 连接到一个远程的机器人控制服务。
    -   连接成功后，它会调用 `session.list_tools()` **获取机器人具备的所有技能（Tools）**，例如 `navigate(destination)`、`grasp(object_id)` 等。
    -   获取技能列表后，它会通过 `collaborator.register_agent()` **向 Master 注册自己**，并上报自己的名称和可用技能列表。

3.  **心跳和监听**:
    -   注册成功后，`RobotManager` 会启动两个关键的后台线程：
        -   **心跳线程 (`_heartbeat_loop`)**: 定期向 Master 发送心跳信号，表明自己仍然在线。
        -   **任务监听线程 (`collaborator.listen`)**: 订阅一个为自己指定的频道（如 `roboos_to_robot_name`），等待 Master 分配子任务。

4.  **任务执行**:
    -   `handle_task()` / `_execute_task()`: 当监听到 Master 发来的子任务时，执行以下操作：
        1.  使用 `ToolMatcher` 从机器人的所有技能中，匹配出与当前子任务最相关的几个技能。
        2.  创建一个临时的 `ToolCallingAgent`。这个 Agent 被赋予了子任务描述、匹配到的技能列表以及 LLM 客户端。
        3.  `agent.run(task)`: Agent 运行后，LLM 会根据子任务描述，决定应该调用哪个技能（Tool），以及传递什么参数。
        4.  通过 `session.call_tool()`，实际调用机器人的技能。
        5.  任务执行完毕后，通过 `collaborator.send()` 将**执行结果**（成功、失败、观察到的信息等）发送回 Master。

---
希望这份文档能帮助您更好地理解 RoboOS 的代码结构！

## 项目概述

RoboOS 是一个**具身智能操作系统**，采用分层分布式架构，用于实现多机器人协作和长时任务规划。

## 核心架构（脑-小脑模式）

```
Deploy (Web UI) - 部署控制器
    ↓
Master (大脑) - 全局任务规划
    ↓ Redis 通信
Slaver (小脑) - 任务执行器
    ↓ MCP 协议
Robot - 物理机器人
```

## 项目结构

```
RoboOS/
├── deploy/          # Web UI 和部署控制器
├── master/          # 大脑 - 全局任务规划器
├── slaver/          # 小脑 - 任务执行器
├── assets/          # 静态资源（logo、图片）
├── FlagScale/       # FlagScale 库占位符
├── test/            # 简单测试脚本
├── TestReal/        # 真实机器人测试组件
└── Test/            # 额外测试配置
```

## 三大核心模块

### 1. Deploy 模块 - Web 部署界面

**目录**: `/home/dora/RoboOS/deploy/`

**技术栈**: Flask Web 框架

**核心文件**:
- `run.py` - 主 Flask 应用，提供 REST API
- `utils.py` - 配置管理辅助函数
- `templates/` - HTML 界面（index.html, deploy.html, release.html）

**功能**:
- 配置管理（读写 YAML 配置）
- 环境验证（端口、Redis、MCP 服务）
- 服务生命周期管理（启动推理、master、slaver）
- Web UI 用于系统监控和任务提交

### 2. Master 模块 - 全局规划器（大脑）

**目录**: `/home/dora/RoboOS/master/`

**技术栈**: Flask + OpenAI API + Redis 协作

**核心文件**:
- `run.py` - Flask 服务器，提供任务发布 API
- `agents/agent.py` - GlobalAgent 类，核心协调器
- `agents/planner.py` - GlobalTaskPlanner，基于 LLM 的任务分解
- `agents/prompts.py` - 任务规划的提示词模板
- `config.yaml` - Master 配置
- `scene/profile.yaml` - 场景/环境定义

**功能**:
- 接收来自 deploy UI 的高层任务
- 使用 LLM (RoboBrain) 将任务分解为子任务
- 将子任务分配给已注册的 slaver 机器人
- 监控机器人状态和任务执行
- 管理场景记忆（物体、位置）

### 3. Slaver 模块 - 执行器（小脑）

**目录**: `/home/dora/RoboOS/slaver/`

**技术栈**: AsyncIO + MCP 协议 + OpenAI API

**核心文件**:
- `run.py` - RobotManager，异步机器人控制器
- `agents/slaver_agent.py` - ToolCallingAgent，ReAct 框架实现
- `agents/models.py` - LLM 客户端封装（OpenAI、Azure）
- `tools/tool_matcher.py` - 使用嵌入的语义工具匹配
- `tools/memory.py` - 智能体和场景记忆管理
- `tools/monitoring.py` - 日志和指标
- `demo_robot_local/skill.py` - 示例机器人技能（MCP 服务器）
- `config.yaml` - Slaver 配置

**功能**:
- 通过 MCP 协议连接机器人（本地或远程）
- 向 master 注册机器人能力
- 监听来自 master 的子任务
- 使用 LLM 选择和执行适当的工具
- 维护场景记忆和机器人状态
- 将执行结果报告回 master

## 已实现的核心功能

### 1. 分层任务规划
- 基于 LLM 的任务分解（master）
- 使用 ReAct 框架的多步任务执行（slaver）
- 顺序和并行任务排序

### 2. 多智能体协作
- 基于 Redis 的 pub/sub 通信（Collaborator）
- 机器人注册和心跳监控
- 基于机器人能力的动态任务分配

### 3. 智能工具匹配
- 使用 sentence transformers 的语义相似度匹配
- 回退到 TF-IDF 和关键词匹配
- 可配置的最大工具数和相似度阈值

### 4. 场景记忆管理
- 符号化场景表示（物体、位置、容器）
- 基于工具执行的自动场景更新
- 基于 LLM 的动作类型预测（添加/移除/移动物体）

### 5. Web 部署界面
- 带验证的配置编辑器
- 服务编排（推理、master、slaver）
- 实时任务提交和监控
- 系统状态仪表板

### 6. MCP 协议集成
- 用于机器人技能通信的模型上下文协议
- 支持本地（stdio）和远程（HTTP）模式
- 动态工具发现和注册

### 7. 错误处理和重试逻辑
- 可配置尝试次数的任务分解重试
- 工具匹配的优雅回退
- 全面的日志和监控

## 技术栈

### 核心技术
- **Python 3.10+** - 主要编程语言
- **Flask** - deploy 和 master 服务的 Web 框架
- **AsyncIO** - slaver 的异步编程
- **Redis** - 智能体协作的消息代理
- **OpenAI API** - LLM 集成（兼容 vLLM）
- **MCP (Model Context Protocol)** - 机器人技能通信

### 关键库
- **fastapi** - REST API 框架
- **flask-socketio** - 实时 Web 通信
- **openai** - LLM 客户端库
- **redis** - Redis 客户端
- **sentence-transformers** - 工具匹配的语义嵌入
- **scikit-learn** - TF-IDF 向量化（回退）
- **pydantic** - 数据验证
- **ruamel.yaml** - 保留注释的 YAML 解析
- **rich** - 终端格式化和日志
- **mcp** - 模型上下文协议实现
- **psutil** - 系统监控

### AI/ML 组件
- **RoboBrain 2.0-7B** - 用于规划和执行的多模态 LLM
- **vLLM** - 高性能 LLM 推理服务器
- **all-MiniLM-L6-v2** - 用于语义匹配的句子转换器

## 系统目的

RoboOS 是一个**具身操作系统**，旨在实现：
- 跨具身机器人协作（单臂、双臂、人形、轮式）
- 长时任务规划和执行
- 复杂环境中的多智能体协调
- 从边缘到云的可扩展部署

## 架构模式：脑-小脑层次结构

```
┌─────────────────────────────────────────────────────────┐
│                    Deploy (Web UI)                      │
│              Configuration & Orchestration              │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Master (Embodied Brain)                    │
│  - Global perception & high-level decision making       │
│  - Task decomposition using MLLM                        │
│  - Multi-agent coordination                             │
└────────────────────┬────────────────────────────────────┘
                     │
                     │ Redis Pub/Sub (Collaborator)
                     │
        ┌────────────┴────────────┬────────────┐
        ▼                         ▼            ▼
┌──────────────┐         ┌──────────────┐  ┌──────────────┐
│   Slaver 1   │         │   Slaver 2   │  │   Slaver N   │
│ (Cerebellum) │         │ (Cerebellum) │  │ (Cerebellum) │
│              │         │              │  │              │
│ - Tool exec  │         │ - Tool exec  │  │ - Tool exec  │
│ - Memory     │         │ - Memory     │  │ - Memory     │
└──────┬───────┘         └──────┬───────┘  └──────┬───────┘
       │                        │                  │
       ▼                        ▼                  ▼
   Robot 1                  Robot 2            Robot N
```

## 关键架构原则

### 1. 关注点分离
- Master: 规划和协调
- Slaver: 执行和控制
- Deploy: 配置和监控

### 2. 可扩展性
- 多个 slaver 可以连接到一个 master
- 基于云的分布式推理
- 边缘-云通信优化

### 3. 模块化
- 即插即用的技能库（Cerebellum）
- 可配置的 LLM 后端
- 灵活的机器人连接模式（本地/远程）

### 4. 实时同步
- 通过 Redis 共享内存
- 心跳监控
- 跨智能体的状态协调

### 5. 鲁棒性
- 规划失败的重试机制
- 工具匹配的回退策略
- 全面的错误处理

## 数据流

```
用户任务 → Deploy UI → Master (LLM 规划) → 子任务 →
Redis 通道 → Slaver (工具选择) → 机器人执行 →
结果 → Master → 场景更新 → 下一个子任务
```

## 总结

RoboOS 是一个成熟的、生产就绪的具身 AI 操作系统，它弥合了高层语言理解和底层机器人控制之间的差距。它实现了受生物系统启发的分层架构（脑-小脑），使用现代 AI 技术（LLM、语义嵌入），并为真实场景中的多机器人协调提供实用工具。代码库结构良好，在规划、执行和部署关注点之间有清晰的分离，使其易于维护和扩展。

---

## 更新记录

**更新时间**: 2026-01-13 07:06:00 UTC

**更新内容**: 添加 Redis 通信和 MCP 协议详细说明

---

# Redis 通信和 MCP 协议详解

基于 RoboOS 的实际代码实现，详细介绍这两个核心通信机制。

## 一、Redis 通信机制（Master ↔ Slaver）

### 1.1 架构角色

Redis 在 RoboOS 中充当**消息代理**，通过 `Collaborator` 类实现 Master 和 Slaver 之间的异步通信。

```
Master (大脑)
    ↓ Redis Pub/Sub
Slaver (小脑)
```

### 1.2 配置信息

**Master 配置** (`master/config.yaml:47-58`):
```yaml
collaborator:
  host: "127.0.0.1"      # Redis 服务器地址
  port: 6379             # Redis 端口
  db: 0                  # 数据库编号
  clear: true            # 启动时清空数据库
  password: 0            # 密码
```

**Slaver 配置** (`slaver/config.yaml:29-40`):
```yaml
collaborator:
  host: "127.0.0.1"
  port: 6379
  db: 0
  clear: false           # Slaver 不清空数据库
  password: 0
```

### 1.3 通信流程

#### **步骤 1: Slaver 注册**

Slaver 启动时向 Master 注册自己的能力：

```python
# slaver/run.py:29
collaborator = Collaborator.from_config(config["collaborator"])

# 注册机器人信息（包括名称、工具列表等）
self.collaborator.register_agent(
    agent_name=self.robot_name,
    agent_info=json.dumps({
        "tools": self.tools,
        "status": "ready"
    })
)
```

#### **步骤 2: Master 监听注册**

Master 监听机器人注册并建立双向通道：

```python
# master/agents/agent.py:78-99
def _handle_register(self, robot_name: str) -> None:
    """监听机器人注册"""
    robot_info = self.collaborator.read_agent_info(robot_name)

    # 创建 Robot → Master 的通道
    channel_r2b = f"{robot_name}_to_RoboOS"
    threading.Thread(
        target=lambda: self.collaborator.listen(
            channel_r2b,
            self._handle_result  # 处理执行结果的回调
        ),
        daemon=True
    ).start()

    self.listening_robots.add(robot_name)
```

#### **步骤 3: Master 发布任务**

Master 通过 Redis 通道发布子任务：

```python
# Master 向特定机器人发送任务
channel_b2r = f"RoboOS_to_{robot_name}"
task_data = {
    "task": "Navigate to kitchen",
    "task_id": "task_001",
    "refresh": False,
    "order_flag": "sequential"
}
self.collaborator.publish(channel_b2r, json.dumps(task_data))
```

#### **步骤 4: Slaver 监听任务**

Slaver 监听自己的任务通道：

```python
# slaver/run.py:257-262
channel_b2r = f"RoboOS_to_{self.robot_name}"
listener_thread = threading.Thread(
    target=lambda: self.collaborator.listen(
        channel_b2r,
        self.handle_task  # 任务处理回调
    ),
    daemon=True
)
listener_thread.start()
```

#### **步骤 5: Slaver 返回结果**

任务执行完成后，Slaver 通过反向通道返回结果：

```python
# Slaver 发送执行结果
channel_r2b = f"{self.robot_name}_to_RoboOS"
result_data = {
    "task_id": "task_001",
    "status": "success",
    "result": "Navigation completed",
    "scene_update": {...}
}
self.collaborator.publish(channel_r2b, json.dumps(result_data))
```

### 1.4 通道命名规范

| 通道名称 | 方向 | 用途 |
|---------|------|------|
| `RoboOS_to_{robot_name}` | Master → Slaver | 任务分发 |
| `{robot_name}_to_RoboOS` | Slaver → Master | 结果上报 |
| `register` | Slaver → Master | 机器人注册 |

### 1.5 数据存储

除了 Pub/Sub，Collaborator 还使用 Redis 存储共享状态：

- **Agent Info**: 机器人能力和状态
- **Environment**: 场景信息（物体、位置）
- **Memory**: 任务执行历史

```python
# 记录场景信息
self.collaborator.record_environment(
    scene_name="kitchen",
    scene_info=json.dumps({
        "objects": ["apple", "cup"],
        "locations": ["table", "counter"]
    })
)
```

---

## 二、MCP 协议（Slaver ↔ Robot）

### 2.1 什么是 MCP？

**MCP (Model Context Protocol)** 是一个开放协议，用于标准化应用程序如何向 LLM 提供上下文。在 RoboOS 中，MCP 用于：

- **工具发现**: 自动发现机器人的可用技能
- **工具调用**: 执行具体的机器人动作
- **结果返回**: 获取执行状态和反馈

### 2.2 两种连接模式

#### **模式 1: Local (stdio)**

适用于本地开发和测试，Slaver 直接启动机器人进程。

**配置** (`slaver/config.yaml:47-53`):
```yaml
robot:
  call_type: local
  path: "demo_robot_local"  # 本地技能文件夹
  name: demo_robot
```

**连接代码** (`slaver/run.py`):
```python
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# 启动本地 MCP 服务器
server_params = StdioServerParameters(
    command="python",
    args=["-m", "demo_robot_local.skill"],
    env=None
)

async with stdio_client(server_params) as (read, write):
    async with ClientSession(read, write) as session:
        await session.initialize()
        # 获取工具列表
        tools = await session.list_tools()
```

#### **模式 2: Remote (HTTP)**

适用于生产环境，机器人运行独立的 MCP 服务器。

**配置**:
```yaml
robot:
  call_type: remote
  path: "http://192.168.1.100:8000"  # 机器人 IP
  name: real_robot
```

**连接代码**:
```python
from mcp.client.streamable_http import streamablehttp_client

async with streamablehttp_client(robot_url) as (read, write):
    async with ClientSession(read, write) as session:
        await session.initialize()
        tools = await session.list_tools()
```

### 2.3 定义机器人技能

使用 FastMCP 框架定义技能（`slaver/demo_robot_local/skill.py:1-43`）:

```python
from mcp.server.fastmcp import FastMCP

# 初始化 MCP 服务器
mcp = FastMCP("robots")

@mcp.tool()
async def navigate_to_target(target: str) -> tuple[str, dict]:
    """导航到目标位置
    Args:
        target: 导航目的地
    """
    result = f"Navigation to {target} has been successfully performed."
    return result, {"position": f"{target}"}

@mcp.tool()
async def grasp_object(object: str) -> tuple[str, dict]:
    """抓取物体
    Args:
        object: 要抓取的物体名称
    """
    result = f"{object} has been successfully grasped."
    return result, {"grasped": f"{object}"}

@mcp.tool()
async def place_to_affordance(affordance: str, object: str = None) -> tuple[str, dict]:
    """放置物体
    Args:
        affordance: 放置位置
        object: 已抓取的物体
    """
    result = f"{object} has been successfully placed on {affordance}."
    return result, {"grasped": "None"}

if __name__ == "__main__":
    # 启动 stdio 传输模式
    mcp.run(transport="stdio")
```

### 2.4 工具调用流程

#### **步骤 1: 工具发现**

Slaver 启动时通过 MCP 获取所有可用工具：

```python
# 初始化 MCP 会话
await session.initialize()

# 列出所有工具
response = await session.list_tools()
self.tools = [
    {
        "type": "function",
        "function": {
            "name": tool.name,
            "description": tool.description,
            "parameters": tool.inputSchema
        }
    }
    for tool in response.tools
]
```

#### **步骤 2: 工具匹配**

收到任务后，使用语义匹配选择相关工具：

```python
# slaver/run.py:123-133
task = "Pick up the apple from the table"

# 使用 ToolMatcher 进行语义匹配
matched_tools = self.tool_matcher.match_tools(task)
# 结果: [("grasp_object", 0.85), ("navigate_to_target", 0.62)]

# 过滤工具列表
filtered_tools = [
    tool for tool in self.tools
    if tool["function"]["name"] in matched_tool_names
]
```

#### **步骤 3: LLM 决策**

ToolCallingAgent 使用 LLM 决定调用哪个工具：

```python
# slaver/agents/slaver_agent.py
agent = ToolCallingAgent(
    tools=filtered_tools,
    model=self.model,
    tool_executor=self.session.call_tool  # MCP 调用接口
)

result = await agent.run("Pick up the apple")
# LLM 输出: {"name": "grasp_object", "arguments": {"object": "apple"}}
```

#### **步骤 4: 执行工具**

通过 MCP 会话执行工具：

```python
# 调用 MCP 工具
result = await session.call_tool(
    name="grasp_object",
    arguments={"object": "apple"}
)

# 返回结果
# result.content = "apple has been successfully grasped."
# result.metadata = {"grasped": "apple"}
```

### 2.5 MCP 协议优势

1. **标准化**: 统一的工具定义和调用接口
2. **解耦**: Slaver 和 Robot 可以独立开发和部署
3. **灵活性**: 支持本地和远程两种模式
4. **可扩展**: 轻松添加新技能，无需修改 Slaver 代码
5. **类型安全**: 自动参数验证和类型检查

---

## 三、完整通信流程示例

```
用户: "把苹果从桌子拿到厨房"
    ↓
Deploy UI → Master
    ↓
Master (LLM 分解任务):
  1. Navigate to table
  2. Grasp apple
  3. Navigate to kitchen
  4. Place apple
    ↓
Redis Pub/Sub: RoboOS_to_demo_robot
    ↓
Slaver 接收任务 1: "Navigate to table"
    ↓
Tool Matcher: 匹配 navigate_to_target (0.92)
    ↓
ToolCallingAgent (LLM):
  → call_tool("navigate_to_target", {"target": "table"})
    ↓
MCP Protocol: session.call_tool()
    ↓
Robot 执行: 导航到桌子
    ↓
MCP 返回: "Navigation to table has been successfully performed."
    ↓
Slaver 更新场景记忆
    ↓
Redis Pub/Sub: demo_robot_to_RoboOS
    ↓
Master 接收结果 → 发送任务 2...
```

---

## 四、关键代码位置

| 功能 | 文件路径 |
|------|---------|
| Redis Collaborator 初始化 | `master/agents/agent.py:21` |
| Master 监听注册 | `master/agents/agent.py:78-99` |
| Slaver 任务监听 | `slaver/run.py:257-262` |
| MCP 连接管理 | `slaver/run.py:32-96` |
| MCP 工具定义 | `slaver/demo_robot_local/skill.py` |
| 工具匹配器 | `slaver/tools/tool_matcher.py` |

这两个协议的结合使 RoboOS 实现了高效的分层通信：**Redis 负责智能体间协作，MCP 负责机器人技能调用**。

