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
