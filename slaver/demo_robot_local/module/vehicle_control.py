"""
车辆控制模块 (Vehicle Control Module)

负责车辆的运动控制，包括转向、油门、刹车等。

Functions:
    - set_vehicle_control: 设置车辆控制命令
    - emergency_brake: 紧急刹车
    - stop_vehicle: 停止车辆
    - move_forward: 前进到指定速度
    - turn_vehicle: 转向到指定角度
"""

import sys
from typing import Tuple, Dict
from vehicle_carla.skills.control import VehicleController

# 全局车辆控制器实例
_vehicle_controller = None


def get_vehicle_controller():
    """获取或创建车辆控制器实例"""
    global _vehicle_controller
    if _vehicle_controller is None:
        _vehicle_controller = VehicleController()
    return _vehicle_controller


def register_tools(mcp):
    """注册车辆控制相关的所有工具函数到 MCP 服务器"""

    @mcp.tool()
    async def set_vehicle_control(steer: float, throttle: float, brake: float) -> Tuple[str, Dict]:
        """Set vehicle control command with steering, throttle, and brake.

        Args:
            steer: Steering angle [-1.0, 1.0] (negative=left, positive=right)
            throttle: Throttle [0.0, 1.0]
            brake: Brake [0.0, 1.0]

        Returns:
            A tuple containing the result message and control state.

        Examples:
            set_vehicle_control(steer=0.0, throttle=0.5, brake=0.0)  # Move forward
            set_vehicle_control(steer=-0.5, throttle=0.3, brake=0.0)  # Turn left
        """
        print(f"[vehicle_control.set_vehicle_control] Setting control: steer={steer}, throttle={throttle}, brake={brake}", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.set_vehicle_control(steer, throttle, brake)

        state_update = {
            "steer": steer,
            "throttle": throttle,
            "brake": brake
        }

        return result, state_update

    @mcp.tool()
    async def emergency_brake() -> Tuple[str, Dict]:
        """Activate emergency brake immediately.

        Returns:
            A tuple containing the result message and control state.

        Examples:
            emergency_brake()
        """
        print("[vehicle_control.emergency_brake] Activating emergency brake", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.emergency_brake()

        state_update = {
            "steer": 0.0,
            "throttle": 0.0,
            "brake": 1.0,
            "emergency": True
        }

        return result, state_update

    @mcp.tool()
    async def stop_vehicle() -> Tuple[str, Dict]:
        """Stop the vehicle smoothly.

        Returns:
            A tuple containing the result message and control state.

        Examples:
            stop_vehicle()
        """
        print("[vehicle_control.stop_vehicle] Stopping vehicle", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.stop_vehicle()

        state_update = {
            "steer": 0.0,
            "throttle": 0.0,
            "brake": 1.0
        }

        return result, state_update

    @mcp.tool()
    async def move_forward(speed: float) -> Tuple[str, Dict]:
        """Move forward at specified speed.

        Args:
            speed: Target speed in m/s

        Returns:
            A tuple containing the result message and control state.

        Examples:
            move_forward(speed=5.0)  # Move at 5 m/s
            move_forward(speed=10.0)  # Move at 10 m/s
        """
        print(f"[vehicle_control.move_forward] Moving forward at speed: {speed} m/s", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.move_forward(speed)

        throttle = min(speed / 10.0, 1.0)
        state_update = {
            "steer": 0.0,
            "throttle": throttle,
            "brake": 0.0,
            "target_speed": speed
        }

        return result, state_update

    @mcp.tool()
    async def move_forward_distance(distance: float, speed: float = 2.0) -> Tuple[str, Dict]:
        """Move forward for specified distance with feedback control.

        Args:
            distance: Target distance in meters
            speed: Speed in m/s (default 2.0)

        Returns:
            A tuple containing the result message and final state.

        Examples:
            move_forward_distance(distance=5.0, speed=2.0)  # Move 5 meters at 2 m/s
        """
        print(f"[vehicle_control.move_forward_distance] Moving {distance}m at {speed}m/s", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.move_forward_distance(distance, speed)

        state_update = {
            "distance_traveled": distance,
            "speed": speed
        }

        return result, state_update

    @mcp.tool()
    async def turn_vehicle(angle: float) -> Tuple[str, Dict]:
        """Turn vehicle to specified angle.

        Args:
            angle: Turn angle in degrees (positive=right, negative=left)

        Returns:
            A tuple containing the result message and control state.

        Examples:
            turn_vehicle(angle=30.0)  # Turn right 30 degrees
            turn_vehicle(angle=-30.0)  # Turn left 30 degrees
        """
        print(f"[vehicle_control.turn_vehicle] Turning vehicle: {angle} degrees", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.turn_vehicle(angle)

        steer = max(-1.0, min(1.0, angle / 45.0))
        state_update = {
            "steer": steer,
            "throttle": 0.3,
            "brake": 0.0,
            "turn_angle": angle
        }

        return result, state_update

    @mcp.tool()
    async def turn_vehicle_angle(target_angle: float, speed: float = 0.3) -> Tuple[str, Dict]:
        """Turn vehicle by specified angle with feedback control.

        Args:
            target_angle: Target turn angle in degrees (positive=right, negative=left)
            speed: Throttle during turn (default 0.3)

        Returns:
            A tuple containing the result message and final state.

        Examples:
            turn_vehicle_angle(target_angle=90.0)  # Turn right 90 degrees
            turn_vehicle_angle(target_angle=-45.0, speed=0.2)  # Turn left 45 degrees slowly
        """
        print(f"[vehicle_control.turn_vehicle_angle] Turning {target_angle}° at speed {speed}", file=sys.stderr)

        controller = get_vehicle_controller()
        result = controller.turn_vehicle_angle(target_angle, speed)

        state_update = {
            "turned_angle": target_angle,
            "turn_speed": speed
        }

        return result, state_update

    print("[vehicle_control.py] 车辆控制模块已注册", file=sys.stderr)
