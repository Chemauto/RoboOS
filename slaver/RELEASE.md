# RoboOS-CARLA 集成开发日志

## 2026-01-19 调试会话总结

### 问题描述

**核心问题**: RoboOS UI发送的车辆控制命令无法控制CARLA中的车辆移动

**症状**:
- RoboOS Agent正确解析工具调用
- VehicleController被调用并报告发送UDP命令成功
- UDP日志显示数据包已发送到127.0.0.1:23456
- 但车辆在CARLA中不移动
- simple_vehicle_control.py没有收到UDP命令(无"[控制指令]"日志)

### 调试过程

#### 1. 工具调用解析问题 (已解决)

**问题**: LLM(robobrain)返回工具调用时,将其放在`content`字段而不是`tool_calls`数组中

**表现**:
```python
ChatCompletionMessage(
    content='{"name": "move_forward", "arguments": {"speed": 5.0}}',
    tool_calls=[]  # 空数组!
)
```

**解决方案**: 修改 `/home/dora/RoboOS/slaver/agents/slaver_agent.py`

**修改位置**: `step()` 方法,第220-244行

**修改内容**:
```python
if model_message.tool_calls:
    tool_call = model_message.tool_calls[0]
    tool_name = tool_call.function.name
    tool_arguments = tool_call.function.arguments
else:
    # 尝试从content中解析工具调用(兼容某些模型)
    try:
        import json
        content = model_message.content.strip()
        if content.startswith('{') and content.endswith('}'):
            tool_data = json.loads(content)
            if 'name' in tool_data and 'arguments' in tool_data:
                tool_name = tool_data['name']
                tool_arguments = tool_data['arguments']
            else:
                return "final_answer"
        else:
            return "final_answer"
    except (json.JSONDecodeError, AttributeError):
        return "final_answer"
```

**结果**: ✅ 工具调用成功解析,日志显示 "Parsed tool call from content: move_forward"

#### 2. 工具参数类型问题 (已解决)

**问题**: `_execute_tool_call()` 和 `memory_predict()` 中对`tool_arguments`调用`json.loads()`,但参数可能已经是dict

**错误信息**:
```
TypeError: the JSON object must be str, bytes or bytearray, not dict
```

**解决方案**: 修改两处代码,检查类型后再解析

**修改位置1**: `/home/dora/RoboOS/slaver/agents/slaver_agent.py` 第158行
```python
# 修改前:
observation = await self.tool_executor(tool_name, json.loads(tool_arguments))

# 修改后:
args = tool_arguments if isinstance(tool_arguments, dict) else json.loads(tool_arguments)
observation = await self.tool_executor(tool_name, args)
```

**修改位置2**: 同文件第194行
```python
# 修改前:
self.scene.apply_action(action_type, json.loads(memory_input["arguments"]))

# 修改后:
args = memory_input["arguments"] if isinstance(memory_input["arguments"], dict) else json.loads(memory_input["arguments"])
self.scene.apply_action(action_type, args)
```

**结果**: ✅ 工具执行成功,无类型错误

#### 3. UDP通信问题 (未完全解决)

**问题**: VehicleController发送UDP包,但simple_vehicle_control.py未接收

**调试步骤**:

1. **添加UDP发送日志**
   - 修改 `/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla/utils/udp_client.py`
   - 添加文件日志 `/tmp/udp_send.log`
   - 添加`reverse`参数以匹配接收端格式

2. **验证UDP发送**
   - 日志确认: UDP包已发送 (80字节到127.0.0.1:23456)
   - 格式正确: `{'id': 'control', 'steer': 0.0, 'throttle': 0.5, 'brake': 0.0, 'reverse': False}`

3. **验证UDP接收**
   - simple_vehicle_control.py进程运行但挂起
   - 日志只有CARLA连接警告,无后续输出
   - 脚本未到达UDP监听阶段
   - 测试UDP接收器也无法接收消息

4. **创建简化脚本**
   - 创建 `/home/dora/RoboOS/simple_udp_vehicle.py`
   - 跳过复杂初始化,直接使用现有车辆
   - 简化版成功启动并显示"[系统] ✓ 就绪!"
   - 但仍未收到UDP测试包

**当前状态**:
- ❌ UDP通信链路未打通
- ✅ 发送端工作正常
- ❌ 接收端存在问题(可能是Python UDP socket配置或系统网络问题)

### 代码修改总结

#### 文件1: `/home/dora/RoboOS/slaver/agents/slaver_agent.py`

**修改1**: 添加content字段工具调用解析 (第220-244行)
**修改2**: 修复_execute_tool_call参数类型处理 (第158-160行)
**修改3**: 修复memory_predict参数类型处理 (第192-195行)

#### 文件2: `/home/dora/RoboOS/slaver/demo_robot_local/vehicle_carla/utils/udp_client.py`

**修改**: 添加reverse参数和详细日志 (第16-42行)
- 添加`reverse: bool = False`参数
- 添加文件日志输出到`/tmp/udp_send.log`
- 添加控制台日志输出
- 使用`flush=True`确保立即输出

#### 文件3: `/home/dora/RoboOS/simple_udp_vehicle.py` (新建)

**用途**: 简化的UDP车辆控制脚本,用于调试UDP通信
**特点**:
- 跳过复杂的车辆spawn流程
- 使用现有CARLA车辆
- 专注于UDP接收和控制应用
- 包含详细的状态日志

#### 文件4: `/home/dora/RoboOS/test_udp_receiver.py` (新建)

**用途**: 最简UDP接收测试脚本
**特点**:
- 纯UDP socket测试
- 无CARLA依赖
- 用于隔离UDP通信问题

### 未解决问题

#### 主要问题: UDP接收端无法接收数据

**现象**:
1. simple_vehicle_control.py启动后挂起,未到达UDP监听阶段
2. 简化脚本simple_udp_vehicle.py启动成功但未收到UDP包
3. 测试脚本test_udp_receiver.py也无法接收UDP包
4. 端口23456未显示在netstat/ss输出中

**可能原因**:
1. Python UDP socket绑定失败(无错误提示)
2. 系统防火墙或网络配置问题
3. Python环境问题(py37 vs roboOS环境)
4. 线程启动问题(daemon线程可能未真正运行)
5. 输出缓冲导致日志延迟

**需要进一步调查**:
- 使用strace追踪socket系统调用
- 检查防火墙规则: `sudo iptables -L`
- 使用tcpdump抓包验证UDP包是否真的发送
- 测试其他端口是否可以正常UDP通信
- 检查Python socket模块是否正常工作

### 测试验证

#### 成功的测试

1. ✅ **工具调用解析**: RoboOS正确解析LLM返回的工具调用
2. ✅ **工具执行**: VehicleController.move_forward()被正确调用
3. ✅ **UDP发送**: UDPClient成功发送80字节数据包
4. ✅ **CARLA连接**: 可以连接CARLA并操作车辆(手动Python脚本)
5. ✅ **车辆生成**: 可以在CARLA中生成和删除车辆

#### 失败的测试

1. ❌ **UDP接收**: simple_vehicle_control.py未收到UDP命令
2. ❌ **车辆移动**: CARLA车辆不响应RoboOS UI命令
3. ❌ **脚本初始化**: simple_vehicle_control.py启动后挂起
4. ❌ **端到端通信**: 完整的RoboOS→UDP→CARLA链路未打通

### 系统状态

**正常运行的组件**:
- CARLA服务器 (PID: 176044)
- RoboOS Master
- RoboOS Slaver
- MCP Server (skill.py)
- VehicleController (发送端)

**问题组件**:
- simple_vehicle_control.py (接收端挂起)
- UDP通信链路 (接收端无响应)

### 下一步建议

#### 短期解决方案

1. **使用tcpdump验证UDP包**:
   ```bash
   sudo tcpdump -i lo -n udp port 23456 -X
   ```

2. **测试基础UDP通信**:
   ```bash
   # 终端1: 使用nc监听
   nc -u -l 23456

   # 终端2: 发送测试数据
   echo "test" | nc -u 127.0.0.1 23456
   ```

3. **检查Python socket**:
   ```python
   import socket
   sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   sock.bind(("0.0.0.0", 23456))
   print("Bound successfully")
   ```

4. **使用strace追踪**:
   ```bash
   strace -e trace=socket,bind,recvfrom python3 simple_udp_vehicle.py
   ```

#### 长期解决方案

1. **重构通信方式**: 考虑使用TCP或其他更可靠的通信方式
2. **添加健康检查**: 在VehicleController中添加UDP连接测试
3. **改进错误处理**: 在UDP发送/接收中添加更详细的错误日志
4. **添加超时机制**: 避免脚本无限挂起

### 关键发现

1. **LLM工具调用格式不一致**: robobrain模型返回格式与标准OpenAI格式不同,需要兼容处理
2. **类型转换问题**: 工具参数可能是dict或JSON字符串,需要统一处理
3. **UDP通信可靠性**: localhost UDP通信也可能出现问题,需要更robust的实现
4. **脚本初始化复杂**: simple_vehicle_control.py初始化过程容易挂起,需要简化

### 经验教训

1. **分层调试**: 从最底层(UDP socket)开始验证,逐层向上
2. **日志重要性**: 文件日志比控制台输出更可靠(避免缓冲问题)
3. **类型检查**: 在处理外部数据时,始终检查类型再操作
4. **简化测试**: 创建最小可复现示例(simple_udp_vehicle.py)
5. **工具验证**: 使用系统工具(netstat, tcpdump)验证网络通信

### 参考资料

- [RoboOS模块化架构文档](demo_robot_local/vehicle_carla/README-master.md)
- [Python socket编程文档](https://docs.python.org/3/library/socket.html)
- [CARLA Python API文档](https://carla.readthedocs.io/en/latest/python_api/)

---

**会话时间**: 2026-01-19 16:00 - 18:00
**调试人员**: Claude Code (Anthropic)
**状态**: 部分解决 (工具调用✅, UDP通信❌)
**下次重点**: 解决UDP接收端问题,打通完整通信链路
