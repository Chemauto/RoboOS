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