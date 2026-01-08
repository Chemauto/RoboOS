# /home/dora/RoboOs/RoboOS/TestReal/demo_robot_local/skill.py
import sys
import time
from typing import Dict, Any

# Add the project root to sys.path to allow importing mcp
sys.path.insert(0, '/home/dora/RoboOs/RoboOS')

from mcp.server.fastmcp import FastMCP

# --- 模拟机器人底层 API ---
class MockLocalRobot:
    def __init__(self):
        self._detected_objects = {
            "water bottle": (2.5, 3.0),
            "apple": (1.0, -0.5),
        }
        self._current_position = (0.0, 0.0)
        self.log("模拟本地机器人已初始化 (using FastMCP)")

    def log(self, message: str):
        """向 stderr 打印日志，这样就不会干扰 stdout 的 MCP 通信"""
        print(f"[MockLocalRobot SKILL] {message}", file=sys.stderr)

    def find_object(self, object_name: str) -> Dict[str, Any]:
        self.log(f"正在寻找 '{object_name}'...")
        time.sleep(1.5)
        if object_name in self._detected_objects:
            coords = self._detected_objects[object_name]
            self.log(f"在坐标 {coords} 处找到 '{object_name}'")
            return {"status": "success", "coords": {"x": coords[0], "y": coords[1]}}
        else:
            self.log(f"未找到 '{object_name}'")
            return {"status": "error", "message": f"Object '{object_name}' not found."}

    def navigate_to_target(self, x: float, y: float) -> Dict[str, Any]:
        self.log(f"收到导航指令: 前往 ({x}, {y})")
        distance = ((self._current_position[0] - x)**2 + (self._current_position[1] - y)**2)**0.5
        duration = distance / 0.5  # 模拟速度
        time.sleep(duration)
        self._current_position = (x, y)
        self.log(f"已到达目标点: {self._current_position}")
        return {"status": "success", "message": f"Navigation to ({x}, {y}) successful."}

# 创建机器人实例
robot = MockLocalRobot()

# --- MCP (机器控制协议) 服务器逻辑 using FastMCP ---
mcp = FastMCP("MockLocalRobot")

@mcp.tool()
async def find_object(object_name: str) -> dict:
    """
    使用摄像头在当前视野中定位一个物体，并返回其二维坐标(x, y)。
    """
    return robot.find_object(object_name)

@mcp.tool()
async def navigate_to_target(x: float, y: float) -> dict:
    """
    命令机器人移动到指定的(x, y)坐标。
    """
    return robot.navigate_to_target(x, y)


if __name__ == "__main__":
    # Initialize and run the server over stdio
    mcp.run(transport="stdio")