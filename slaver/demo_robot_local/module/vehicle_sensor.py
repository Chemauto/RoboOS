"""
车辆传感器模块 (Vehicle Sensor Module)

负责读取车辆传感器数据，包括GNSS、IMU等。

Functions:
    - get_gnss_data: 获取GPS位置数据
    - get_imu_data: 获取IMU数据
    - get_vehicle_status: 获取车辆状态
"""

import sys
from typing import Tuple, Dict
from vehicle_carla.skills.sensor import SensorReader

# 全局传感器读取器实例
_sensor_reader = None


def get_sensor_reader():
    """获取或创建传感器读取器实例"""
    global _sensor_reader
    if _sensor_reader is None:
        _sensor_reader = SensorReader()
    return _sensor_reader


def register_tools(mcp):
    """注册车辆传感器相关的所有工具函数到 MCP 服务器"""

    @mcp.tool()
    async def get_gnss_data() -> Tuple[str, Dict]:
        """Get GPS position data from vehicle.

        Returns:
            A tuple containing the GNSS position message and sensor data.

        Examples:
            get_gnss_data()
        """
        print("[vehicle_sensor.get_gnss_data] Reading GNSS data", file=sys.stderr)

        sensor_reader = get_sensor_reader()
        result = sensor_reader.get_gnss_data()

        state_update = {
            "gnss_data": sensor_reader.latest_gnss if sensor_reader.latest_gnss else {}
        }

        return result, state_update

    @mcp.tool()
    async def get_imu_data() -> Tuple[str, Dict]:
        """Get IMU data from vehicle (acceleration, angular velocity, heading).

        Returns:
            A tuple containing the IMU data message and sensor data.

        Examples:
            get_imu_data()
        """
        print("[vehicle_sensor.get_imu_data] Reading IMU data", file=sys.stderr)

        sensor_reader = get_sensor_reader()
        result = sensor_reader.get_imu_data()

        state_update = {
            "imu_data": sensor_reader.latest_imu if sensor_reader.latest_imu else {}
        }

        return result, state_update

    @mcp.tool()
    async def get_vehicle_status() -> Tuple[str, Dict]:
        """Get comprehensive vehicle status (position, speed, heading).

        Returns:
            A tuple containing the vehicle status message and sensor data.

        Examples:
            get_vehicle_status()
        """
        print("[vehicle_sensor.get_vehicle_status] Reading vehicle status", file=sys.stderr)

        sensor_reader = get_sensor_reader()
        result = sensor_reader.get_vehicle_status()

        state_update = {
            "gnss_data": sensor_reader.latest_gnss if sensor_reader.latest_gnss else {},
            "imu_data": sensor_reader.latest_imu if sensor_reader.latest_imu else {}
        }

        return result, state_update

    print("[vehicle_sensor.py] 车辆传感器模块已注册", file=sys.stderr)
