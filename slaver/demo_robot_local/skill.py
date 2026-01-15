from mcp.server.fastmcp import FastMCP
from vehicle_carla.skills.control import VehicleController
from vehicle_carla.skills.sensor import SensorReader
from vehicle_carla.skills.simulation import SimulationManager

# Initialize FastMCP server
mcp = FastMCP("robots")

# Initialize vehicle_carla modules
controller = VehicleController()
sensor = SensorReader()
simulation = SimulationManager()


@mcp.tool()
async def navigate_to_target(target: str) -> tuple[str, dict]:
    """Navigate to target, do not call when Navigation to target has been successfully performed.
    Args:
        target: String, Represents the navigation destination.
    """
    result = f"Navigation to {target} has been successfully performed."
    print(result)
    return result, {"position": f"{target}"}


@mcp.tool()
async def grasp_object(object: str) -> tuple[str, dict]:
    """Pick up the object, do not call when object has been successfully grasped.
    Args:
        object: String, Represents which to grasp.
    """
    result = f"{object} has been successfully grasped."
    print(result)
    return result, {"grasped": f"{object}"}


@mcp.tool()
async def place_to_affordance(affordance: str, object: str = None) -> tuple[str, dict]:
    """Place the grasped object in affordance, do not call when object has been successfully placed on affordance."
    Args:
        affordance: String, Represents where the object to place.
        object: String, Represents the object has been grasped.
    """
    result = f"{object} has been successfully placed on {affordance}."
    print(result)
    return result, {"grasped": f"None"}


# ========== Vehicle CARLA Skills ==========
# 以下是 Vehicle CARLA 仿真系统的 11 个 skills
# 用于在 CARLA 仿真器中控制车辆、获取传感器数据和管理仿真

# ===== 车辆控制类 Skills (5个) =====

@mcp.tool()
async def set_vehicle_control(steer: float, throttle: float, brake: float) -> str:
    """设置车辆控制指令

    功能：通过 UDP 发送控制命令到 CARLA 仿真器，控制车辆的转向、油门和制动
    使用场景：需要精确控制车辆行为时使用

    Args:
        steer: 转向角 [-1.0, 1.0]，负值=左转，正值=右转
        throttle: 油门 [0.0, 1.0]，0=无油门，1=最大油门
        brake: 制动 [0.0, 1.0]，0=无制动，1=最大制动

    Returns:
        执行状态消息
    """
    return controller.set_vehicle_control(steer, throttle, brake)


@mcp.tool()
async def emergency_brake() -> str:
    """紧急制动

    功能：立即停止车辆，发送最大制动指令
    使用场景：紧急情况下需要立即停车

    Returns:
        制动状态消息
    """
    return controller.emergency_brake()


@mcp.tool()
async def stop_vehicle() -> str:
    """平稳停车

    功能：平稳地停止车辆
    使用场景：正常停车时使用

    Returns:
        停车状态消息
    """
    return controller.stop_vehicle()


@mcp.tool()
async def move_forward(speed: float) -> str:
    """前进到指定速度

    功能：控制车辆以指定速度前进
    使用场景：需要车辆匀速前进时使用

    Args:
        speed: 目标速度，单位 m/s（米/秒）

    Returns:
        当前速度状态消息
    """
    return controller.move_forward(speed)


@mcp.tool()
async def turn_vehicle(angle: float) -> str:
    """转向到指定角度

    功能：控制车辆转向到指定角度
    使用场景：需要车辆转弯时使用

    Args:
        angle: 转向角度（度），正值=右转，负值=左转

    Returns:
        当前转向角消息
    """
    return controller.turn_vehicle(angle)


# ===== 传感器数据类 Skills (3个) =====

@mcp.tool()
async def get_gnss_data() -> str:
    """获取 GPS 位置数据

    功能：从 CARLA 仿真器接收 GNSS（GPS）传感器数据
    使用场景：需要获取车辆当前位置坐标时使用

    Returns:
        x, y, z 坐标信息
    """
    return sensor.get_gnss_data()


@mcp.tool()
async def get_imu_data() -> str:
    """获取 IMU 数据

    功能：从 CARLA 仿真器接收 IMU（惯性测量单元）传感器数据
    使用场景：需要获取车辆加速度、角速度和航向角时使用

    Returns:
        加速度、角速度、航向角信息
    """
    return sensor.get_imu_data()


@mcp.tool()
async def get_vehicle_status() -> str:
    """获取车辆综合状态

    功能：获取车辆的综合状态信息（位置、速度、航向）
    使用场景：需要全面了解车辆当前状态时使用

    Returns:
        位置、速度、航向综合信息
    """
    return sensor.get_vehicle_status()


# ===== 仿真管理类 Skills (3个) =====

@mcp.tool()
async def start_carla_simulation(scenario: str = "default") -> str:
    """启动 CARLA 仿真

    功能：启动 CARLA 仿真器并加载指定场景
    使用场景：开始仿真测试前使用

    Args:
        scenario: 场景名称（可选，默认="default"）

    Returns:
        仿真启动状态消息
    """
    return simulation.start_carla_simulation(scenario)


@mcp.tool()
async def stop_carla_simulation() -> str:
    """停止 CARLA 仿真

    功能：停止当前运行的 CARLA 仿真
    使用场景：仿真测试结束后使用

    Returns:
        仿真停止状态消息
    """
    return simulation.stop_carla_simulation()


@mcp.tool()
async def get_simulation_status() -> str:
    """获取仿真状态

    功能：查询当前仿真的运行状态和时间信息
    使用场景：需要检查仿真是否正在运行时使用

    Returns:
        运行状态和时间戳信息
    """
    return simulation.get_simulation_status()


if __name__ == "__main__":
    # Initialize and run the server
    mcp.run(transport="stdio")
