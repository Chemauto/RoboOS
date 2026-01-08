# /home/dora/RoboOs/RoboOS/TestReal/mock_robot_api.py
# 这是一个模拟的机器人底层API库
# 在真实场景中，您应该用您机器人的真实Python SDK替换这里的逻辑

import time
import random

class MockCamera:
    """模拟一个摄像头"""
    def __init__(self):
        # 假装数据库里有一些物体和它们的坐标
        # Key 是物体名称，Value 是 (x, y) 坐标
        self._detected_objects = {
            "water bottle": (2.5, 3.0),
            "apple": (1.0, -0.5),
            "book": (2.8, 2.9),
            "keyboard": (-1.2, 0.8),
        }
        print(" [Mock Robot API] 摄像头已初始化")

    def find(self, object_name: str):
        """模拟在画面中寻找物体"""
        print(f" [Mock Robot API] 正在用摄像头寻找 '{object_name}'...")
        time.sleep(1.5) # 模拟处理时间

        # 模拟有一定概率找不到物体
        if random.random() < 0.1: # 10% 概率找不到
            print(f" [Mock Robot API] 未在画面中找到 '{object_name}'")
            return None
        
        # 模拟物体位置每次都有点轻微变化
        if object_name in self._detected_objects:
            base_coords = self._detected_objects[object_name]
            noise_x = random.uniform(-0.1, 0.1)
            noise_y = random.uniform(-0.1, 0.1)
            found_coords = (round(base_coords[0] + noise_x, 2), round(base_coords[1] + noise_y, 2))
            print(f" [Mock Robot API] 成功在坐标 {found_coords} 处找到 '{object_name}'")
            return {"x": found_coords[0], "y": found_coords[1]}
        
        return None


class MockChassis:
    """模拟一个移动底盘"""
    def __init__(self):
        self._current_position = (0.0, 0.0)
        print(" [Mock Robot API] 移动底盘已初始化")

    def go_to(self, x: float, y: float):
        """模拟移动到指定的(x, y)坐标"""
        print(f" [Mock Robot API] 收到导航指令: 前往 ({x}, {y})")
        print(f" [Mock Robot API] 当前位置: {self._current_position}")
        
        # 模拟导航过程
        distance = ((self._current_position[0] - x)**2 + (self._current_position[1] - y)**2)**0.5
        duration = distance / 0.5 # 假设速度是 0.5m/s
        print(f" [Mock Robot API] 路径规划中... 预计耗时 {duration:.2f} 秒")
        time.sleep(duration)
        
        self._current_position = (x, y)
        print(f" [Mock Robot API] 已到达目标点: {self._current_position}")
        return True


class MockRobot:
    """将所有模拟组件组合成一个机器人对象"""
    def __init__(self):
        self.camera = MockCamera()
        self.chassis = MockChassis()
        print(" [Mock Robot API] 模拟机器人已上线")

# 创建一个全局的机器人实例，这样服务在多次调用之间可以共享状态
# 在真实的应用中，这代表了与机器人硬件的持久连接
robot_instance = MockRobot()
