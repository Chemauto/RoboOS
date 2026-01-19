"""
CARLA仿真管理模块 (CARLA Simulation Management Module)

负责CARLA仿真环境的启动、停止和状态查询。

Functions:
    - start_carla_simulation: 启动CARLA仿真
    - stop_carla_simulation: 停止CARLA仿真
    - get_simulation_status: 获取仿真状态
"""

import sys
from typing import Tuple, Dict
from vehicle_carla.skills.simulation import SimulationManager

# 全局仿真管理器实例
_sim_manager = None


def get_sim_manager():
    """获取或创建仿真管理器实例"""
    global _sim_manager
    if _sim_manager is None:
        _sim_manager = SimulationManager()
    return _sim_manager


def register_tools(mcp):
    """注册CARLA仿真相关的所有工具函数到 MCP 服务器"""

    @mcp.tool()
    async def start_carla_simulation(scenario: str = "default") -> Tuple[str, Dict]:
        """Start CARLA simulation with specified scenario.

        Args:
            scenario: Scenario name (default: "default")

        Returns:
            A tuple containing the result message and simulation state.

        Examples:
            start_carla_simulation(scenario="default")
            start_carla_simulation(scenario="urban")
        """
        print(f"[vehicle_simulation.start_carla_simulation] Starting simulation with scenario: {scenario}", file=sys.stderr)

        sim_manager = get_sim_manager()
        result = sim_manager.start_carla_simulation(scenario)

        state_update = {
            "simulation_running": sim_manager.simulation_running,
            "scenario": scenario
        }

        return result, state_update

    @mcp.tool()
    async def stop_carla_simulation() -> Tuple[str, Dict]:
        """Stop CARLA simulation.

        Returns:
            A tuple containing the result message and simulation state.

        Examples:
            stop_carla_simulation()
        """
        print("[vehicle_simulation.stop_carla_simulation] Stopping simulation", file=sys.stderr)

        sim_manager = get_sim_manager()
        result = sim_manager.stop_carla_simulation()

        state_update = {
            "simulation_running": sim_manager.simulation_running
        }

        return result, state_update

    @mcp.tool()
    async def get_simulation_status() -> Tuple[str, Dict]:
        """Get current simulation status.

        Returns:
            A tuple containing the status message and simulation state.

        Examples:
            get_simulation_status()
        """
        print("[vehicle_simulation.get_simulation_status] Checking simulation status", file=sys.stderr)

        sim_manager = get_sim_manager()
        result = sim_manager.get_simulation_status()

        state_update = {
            "simulation_running": sim_manager.simulation_running
        }

        return result, state_update

    print("[vehicle_simulation.py] CARLA仿真管理模块已注册", file=sys.stderr)
