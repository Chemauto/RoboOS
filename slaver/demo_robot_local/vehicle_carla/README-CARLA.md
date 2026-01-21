# 问题记录

## 2026-01-21: 工具调用格式问题

### 问题描述

模型"robobrain"不能正确支持OpenAI的原生工具调用格式。当要求调用工具时，它输出的是文本格式如`(get_current_heading())`，而不是结构化的tool_calls。

### 问题表现

**模型输出：**
```
(get_current_heading())
<tool_call>
```

**API响应：**
```python
tool_calls=[]  # 空数组，没有原生工具调用
content='(get_current_heading())\n<tool_call>\n'  # 文本内容
```

### 已实施的修复

1. **禁用原生工具调用**（`slaver/config.yaml`）
   ```yaml
   tool:
     support_tool_calls: false  # 从 true 改为 false
   ```

2. **添加文本解析器**（`slaver/agents/models.py:212-231`）
   - 使用正则表达式从文本中提取`(function_name())`格式
   - 模式：`r'\(([a-zA-Z_][a-zA-Z0-9_]*)\(\)\)'`
   - 将文本格式转换为结构化的ToolCall对象

3. **调整token限制**（`slaver/agents/models.py:378`）
   ```python
   max_tokens: 6000  # 从 8192 减少到 6000
   ```

### 当前状态

- ✅ 模型能够输出函数调用文本格式
- ✅ 正则解析器已添加
- ⚠️ 需要验证函数是否实际执行
- ⚠️ 日志中未见明确的执行结果

### 可能的问题

1. **正则表达式匹配问题**
   - 模式期望`(function_name())`
   - 模型输出包含额外内容如`<tool_call>`标签

2. **执行结果未记录**
   - 日志显示输入但没有输出/观察结果
   - 可能函数执行了但结果没有被正确记录

3. **工具执行错误**
   - `convert_chat_message`执行期间可能有未记录的错误

### 验证方法

检查以下内容确认是否工作：
- [ ] 函数是否真的被调用（查看UDP数据包发送到CARLA）
- [ ] 工具调用后是否有"Observation:"或结果消息
- [ ] `convert_chat_message`执行期间是否发生错误
- [ ] 查看`/home/dora/RoboOS/slaver/.log/agent.log`中的完整执行流程

### 相关文件

- `slaver/config.yaml` - 工具调用配置
- `slaver/agents/models.py` - 模型和解析器实现
- `slaver/.log/agent.log` - 执行日志

### 测试指令

```
调用get_current_heading函数以获取当前车辆朝向
```

**预期行为：**
- 模型输出：`(get_current_heading())`
- 解析器提取：`get_current_heading`
- 执行函数并返回车辆朝向角度

---


```bash
conda activate roboOS
unset all_proxy
unset ALL_PROXY
cd deploy/
bash start_serve.sh 
```
```bash
conda activate roboOS
unset all_proxy
unset ALL_PROXY
cd slaver/
python run.py 
```
```bash
conda activate roboOS
unset all_proxy
unset ALL_PROXY
python run.py 

```bash
conda activate roboOS
cd deploy/
python run.py 
```

```bash
cd RoboOS/Vehicle/CARLA_Leaderboard_20/
bash CarlaUE4.sh
```

```bash
cd RoboOS
python simple_vehicle_control.py 
```

RoboOS/slaver/demo_robot_local/vehicle_carla_test/README_test.md验证了1-7的功能是可以的

