# Vehicle-CARLA集成到RoboOS的Skill系统

**创建时间**: 2026-01-15 13:40 CST
**最后更新**: 2026-01-15 13:50 CST
**更新内容**: 明确只针对CARLA仿真（不连接硬件）的skills列表

## 项目背景

将Vehicle项目（自动驾驶车辆系统）的功能集成到RoboOS的skill系统中，通过UI界面发送指令，调用Vehicle中的skills，实现在CARLA仿真器中的自动驾驶仿真。

## Vehicle系统功能分析

基于 `Vehicle/README-master.md` 的分析，Vehicle系统支持两种运行模式：

### 1. 硬件模式
- 使用真实的Livox MID360激光雷达
- 车辆控制器硬件接口
- 完整的自主驾驶功能栈

### 2. CARLA仿真模式（推荐集成）
- 在CARLA 0.9.14仿真器中运行
- 无需物理硬件
- 完整的传感器仿真（GNSS/IMU/LiDAR）
- UDP/TCP通信接口

## 可注册为RoboOS Skills的功能（仅CARLA仿真）

> **重要说明**: 以下skills仅适用于CARLA仿真模式，不需要物理硬件支持。

### ✅ 必需Skills（核心功能）

#### 🚗 车辆控制类 (Vehicle Control)

| Skill名称 | 功能描述 | 参数 | 返回值 | 优先级 |
|----------|---------|------|--------|--------|
| `set_vehicle_control` | 设置车辆控制指令 | steer: float [-1.0, 1.0]<br>throttle: float [0.0, 1.0]<br>brake: float [0.0, 1.0] | 执行状态 | ⭐⭐⭐ 高 |
| `emergency_brake` | 紧急制动 | 无 | 制动状态 | ⭐⭐⭐ 高 |
| `move_forward` | 前进到指定速度 | speed: float (m/s) | 当前速度 | ⭐⭐ 中 |
| `turn_vehicle` | 转向到指定角度 | angle: float (度) | 当前转向角 | ⭐⭐ 中 |
| `stop_vehicle` | 停止车辆 | 无 | 停止状态 | ⭐⭐⭐ 高 |

#### 📡 传感器数据类 (Sensor Data)

| Skill名称 | 功能描述 | 参数 | 返回值 | 优先级 |
|----------|---------|------|--------|--------|
| `get_gnss_data` | 获取GPS位置数据 | 无 | x, y, z坐标 | ⭐⭐⭐ 高 |
| `get_imu_data` | 获取IMU数据 | 无 | 加速度、角速度、航向角 | ⭐⭐ 中 |
| `get_vehicle_status` | 获取车辆状态 | 无 | 位置、速度、航向 | ⭐⭐⭐ 高 |

#### 🎮 仿真管理类 (Simulation Management)

| Skill名称 | 功能描述 | 参数 | 返回值 | 优先级 |
|----------|---------|------|--------|--------|
| `start_carla_simulation` | 启动CARLA仿真 | scenario: str (可选) | 仿真状态 | ⭐⭐⭐ 高 |
| `stop_carla_simulation` | 停止CARLA仿真 | 无 | 停止状态 | ⭐⭐⭐ 高 |
| `get_simulation_status` | 获取仿真状态 | 无 | 运行状态、时间戳 | ⭐⭐ 中 |

### ⚠️ 排除的Skills（需要硬件支持）

以下功能需要物理硬件，**不在CARLA仿真模式中实现**：

- ❌ `perform_slam_localization` - 需要真实激光雷达
- ❌ `detect_obstacles` - 需要真实传感器（CARLA已内置）
- ❌ `filter_ground_points` - 需要真实点云处理
- ❌ `broadcast_pose` - 多车协同功能（暂不实现）
- ❌ `get_lidar_data` - CARLA点云数据量大，暂不实现

### 📋 最终Skills清单（共11个）

**阶段1 - 基础控制（5个）**:
1. `set_vehicle_control(steer, throttle, brake)` - 设置车辆控制
2. `emergency_brake()` - 紧急制动
3. `stop_vehicle()` - 停止车辆
4. `get_vehicle_status()` - 获取车辆状态
5. `get_gnss_data()` - 获取GPS位置

**阶段2 - 高级控制（3个）**:
6. `move_forward(speed)` - 前进到指定速度
7. `turn_vehicle(angle)` - 转向到指定角度
8. `get_imu_data()` - 获取IMU数据

**阶段3 - 仿真管理（3个）**:
9. `start_carla_simulation(scenario)` - 启动仿真
10. `stop_carla_simulation()` - 停止仿真
11. `get_simulation_status()` - 获取仿真状态

## CARLA仿真通信架构

### 数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                    CARLA 仿真器                              │
│  - 生成传感器数据 (GNSS/IMU/LiDAR)                          │
│  - 执行车辆控制指令                                          │
└────────────┬────────────────────────────────┬───────────────┘
             │ 传感器数据                      │ 控制指令
             ↓                                 ↑
┌─────────────────────────────────────────────────────────────┐
│              Leaderboard 评估框架                            │
│  - 调用 agent.run_step(input_data, timestamp)               │
└────────────┬────────────────────────────────┬───────────────┘
             │ 传感器数据                      │ 控制指令
             ↓                                 ↑
┌─────────────────────────────────────────────────────────────┐
│                   dora.py Agent                              │
│  - 接收CARLA传感器数据                                       │
│  - 通过UDP发送传感器数据                                     │
│  - 通过UDP接收控制命令                                       │
└────────────┬────────────────────────────────┬───────────────┘
             │ UDP/TCP                         │ UDP
             ↓                                 ↑
┌─────────────────────────────────────────────────────────────┐
│              RoboOS Skill System (新增)                      │
│  - 接收UI指令                                                │
│  - 调用Vehicle skills                                        │
│  - 发送控制命令到CARLA                                       │
└─────────────────────────────────────────────────────────────┘
```

### 网络配置

**传感器数据发送**:
- GNSS/IMU: UDP → 192.168.1.101:12345
- LiDAR: TCP → 192.168.1.101:5005

**控制命令接收**:
- 控制指令: UDP ← 192.168.1.1:23456

**控制命令格式**:
```json
{
  "id": "control",
  "steer": 0.5,      // 转向角 [-1.0, 1.0]
  "throttle": 0.3,   // 油门 [0.0, 1.0]
  "brake": 0.0       // 制动 [0.0, 1.0]
}
```

## 集成方案设计

### 方案选择：CARLA仿真模式

**选择理由**:
1. ✅ 无需物理硬件，易于开发和测试
2. ✅ 已有完整的UDP通信接口
3. ✅ 文档完善，集成路径清晰
4. ✅ 支持多种测试场景

### 集成架构

```
RoboOS UI (Deploy)
    ↓ HTTP
RoboOS Master (任务分解)
    ↓ Redis Pub/Sub
RoboOS Slaver (vehicle_carla)
    ↓ MCP Protocol
Vehicle CARLA Skills (skill.py)
    ↓ UDP/TCP
CARLA Simulator (dora.py agent)
```

### 实现步骤

#### 阶段1: 基础Skill注册（优先级：高）
1. 创建 `slaver/vehicle_carla/` 目录
2. 实现 `skill.py` 注册基础控制skills
3. 实现UDP通信接口
4. 测试基本的车辆控制功能

**基础Skills**:
- `set_vehicle_control(steer, throttle, brake)`
- `get_vehicle_status()`
- `emergency_brake()`

#### 阶段2: 传感器数据集成（优先级：中）
1. 实现传感器数据接收
2. 注册传感器数据获取skills
3. 数据格式转换和处理

**传感器Skills**:
- `get_gnss_data()`
- `get_imu_data()`
- `get_lidar_data()`

#### 阶段3: 高级任务执行（优先级：中）
1. 实现路径规划接口
2. 实现导航功能
3. 集成避障功能

**任务Skills**:
- `navigate_to_waypoint(x, y)`
- `follow_route(waypoints)`
- `avoid_obstacle(obstacle_info)`

#### 阶段4: 仿真管理（优先级：低）
1. 实现CARLA仿真启动/停止
2. 场景配置管理
3. 性能监控

**管理Skills**:
- `start_simulation(scenario)`
- `stop_simulation()`
- `get_simulation_status()`

## 技术要点

### 1. MCP协议集成
- 使用FastMCP框架定义skills
- 支持本地(stdio)模式
- 异步函数实现

### 2. UDP通信
- 使用Python socket库
- JSON格式数据传输
- 非阻塞接收

### 3. 数据同步
- 传感器数据缓存
- 控制命令队列
- 状态同步机制

### 4. 错误处理
- 网络连接异常
- CARLA服务器断开
- 超时处理

## 配置文件

### slaver/config.yaml 配置
```yaml
robot:
  call_type: local
  path: "vehicle_carla"
  name: vehicle_carla

# Vehicle CARLA 特定配置
vehicle_carla:
  carla_host: "192.168.1.1"
  carla_port: 23456
  sensor_host: "192.168.1.101"
  gnss_port: 12345
  lidar_port: 5005
```

## 测试计划

### 单元测试
- [ ] UDP通信测试
- [ ] 控制命令格式测试
- [ ] 传感器数据解析测试

### 集成测试
- [ ] RoboOS → Vehicle skill调用测试
- [ ] Vehicle skill → CARLA通信测试
- [ ] 完整数据流测试

### 功能测试
- [ ] 基础车辆控制（前进、转向、制动）
- [ ] 传感器数据获取
- [ ] 导航到指定点
- [ ] 路径跟随

### 性能测试
- [ ] 控制延迟测试
- [ ] 数据传输速率测试
- [ ] 系统稳定性测试

## 依赖环境

### CARLA环境
- CARLA 0.9.14
- Python 3.7 (conda环境)
- scenario_runner
- leaderboard

### RoboOS环境
- Python 3.10+
- FastMCP
- Redis
- OpenAI API (LLM)

### Python包
- open3d (点云处理)
- carla (CARLA Python API)
- socket (UDP通信)
- json (数据格式)

## 预期效果

### 用户体验
1. 在RoboOS Deploy UI输入指令："让车辆前进到路口"
2. Master分解任务：
   - 获取当前位置
   - 规划路径到路口
   - 执行导航
3. Slaver调用vehicle_carla skills
4. CARLA仿真器中车辆执行动作
5. 返回执行结果到UI

### 示例指令
- "启动CARLA仿真"
- "让车辆前进10米"
- "转向左侧30度"
- "导航到坐标(100, 200)"
- "紧急制动"
- "获取当前位置"

## 下一步行动

1. **创建skill模块结构** (30分钟)
   - 创建目录和基础文件
   - 配置MCP服务器

2. **实现基础控制skills** (2小时)
   - UDP通信实现
   - 基础控制命令
   - 测试验证

3. **集成到RoboOS** (1小时)
   - 配置slaver
   - 测试skill注册
   - 验证调用流程

4. **编写使用文档** (30分钟)
   - 使用示例
   - 配置说明
   - 故障排查

## 参考资料

- Vehicle/README-master.md - Vehicle系统完整文档
- Node.md - RoboOS系统架构文档
- slaver/demo_robot_local/skill.py - Skill实现示例

---

**状态**: 📝 规划阶段
**负责人**: SunSunSun689
**协作**: Claude Sonnet 4.5
