# /home/dora/RoboOs/RoboOS/TestReal/real_skill_server.py
# ----------------- 这是您最需要关注和修改的文件 -----------------
#
# 它是一个 Flask Web 服务器，充当了实体机器人的"技能接口"。
# RoboOS Slaver 会通过网络请求(HTTP)来调用这个文件里定义的技能。
#
# 如何运行这个文件:
# 1. 确保安装了 flask: pip install flask
# 2. 在命令行中直接运行: python /home/dora/RoboOs/RoboOS/TestReal/real_skill_server.py
#
# -----------------------------------------------------------------

from flask import Flask, request, jsonify
import json

# 从我们模拟的机器人API库中导入机器人实例
# 在真实场景中，这里会导入您自己编写的、控制真实机器人的代码
from mock_robot_api import robot_instance 

# 初始化 Flask 应用
app = Flask(__name__)

# --- 定义机器人拥有的技能 ---
# 这是一个技能列表，Slaver会首先请求这个列表来了解机器人能做什么
# 每个技能都应该有清晰的'name'和'description'，这样LLM才能理解并正确调用它
# 'input_schema' 定义了调用该技能需要哪些参数
robot_tools = [
    {
        "type": "function",
        "function": {
            "name": "find_object",
            "description": "使用摄像头在当前视野中定位一个物体，并返回其二维坐标(x, y)。当你需要知道某个物体在哪里时调用此工具。",
            "input_schema": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "需要寻找的物体的名称, 例如: 'a red apple', 'the water bottle'."
                    }
                },
                "required": ["object_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_target",
            "description": "命令机器人移动到指定的(x, y)坐标。这个工具只接受精确的坐标作为参数。",
            "input_schema": {
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "description": "目标点的 X 坐标。"
                    },
                    "y": {
                        "type": "number",
                        "description": "目标点的 Y 坐标。"
                    }
                },
                "required": ["x", "y"]
            }
        }
    }
]

# --- 实现技能的具体逻辑 ---
# 将技能名称映射到具体的Python函数
# ***** 这是您需要用真实机器人代码替换的核心部分 *****
def find_object(object_name: str):
    """
    调用模拟摄像头的find方法。
    在真实世界中，这里会是复杂的计算机视觉代码。
    """
    coords = robot_instance.camera.find(object_name)
    if coords:
        return f"成功在坐标 {coords} 处找到 {object_name}。"
    else:
        return f"错误: 在视野中没有找到 {object_name}。"

def navigate_to_target(x: float, y: float):
    """
    调用模拟底盘的go_to方法。
    在真实世界中，这里会是调用机器人导航系统的代码。
    """
    success = robot_instance.chassis.go_to(x, y)
    if success:
        return f"已成功导航到坐标 ({x}, {y})。"
    else:
        return f"错误: 导航到坐标 ({x}, {y}) 失败。"

# 将技能名称映射到函数
available_functions = {
    "find_object": find_object,
    "navigate_to_target": navigate_to_target,
}


# --- 创建Web服务器API接口 ---
# Slaver会通过访问这些URL来与机器人交互

@app.route("/")
def index():
    return "真实机器人技能服务器已上线！"

# Slaver会调用这个接口来获取机器人能做什么
@app.route("/list_tools", methods=['GET'])
def list_tools():
    print("[Skill Server] 收到了 /list_tools 请求，返回技能列表。")
    return jsonify(robot_tools)

# Slaver会调用这个接口来执行具体的技能
@app.route("/call_tool", methods=['POST'])
def call_tool():
    data = request.json
    tool_name = data.get("name")
    params = data.get("params", {})
    
    print(f"[Skill Server] 收到了 /call_tool 请求: 执行 '{tool_name}'，参数: {params}")

    if tool_name in available_functions:
        function_to_call = available_functions[tool_name]
        try:
            # 调用对应的Python函数，并将参数解包传入
            result = function_to_call(**params)
            print(f"[Skill Server] 技能 '{tool_name}' 执行结果: {result}")
            return jsonify({"status": "success", "result": result})
        except Exception as e:
            error_message = f"执行技能 '{tool_name}' 时发生错误: {e}"
            print(f"[Skill Server] {error_message}")
            return jsonify({"status": "error", "message": error_message}), 500
    else:
        error_message = f"错误: 未知的技能 '{tool_name}'"
        print(f"[Skill Server] {error_message}")
        return jsonify({"status": "error", "message": error_message}), 404

@app.route("/mcp", methods=['GET', 'POST'])
def mcp_endpoint():
    """模拟一个 MCP 端点，用于 Slaver 的连接检查"""
    return jsonify({}), 200

if __name__ == '__main__':
    # 启动服务器，监听在 5001 端口
    # 确保这个端口没有被其他程序占用
    app.run(host='0.0.0.0', port=5001)
