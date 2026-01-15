"""Simulation Management Skills"""
import subprocess
import time
from typing import Optional
from ..utils.logger import setup_logger

logger = setup_logger("vehicle_simulation")


class SimulationManager:
    """Simulation management functionality"""

    def __init__(self):
        self.simulation_process: Optional[subprocess.Popen] = None
        self.simulation_running = False
        self.start_time: Optional[float] = None
        logger.info("SimulationManager initialized")

    def start_carla_simulation(self, scenario: str = "default") -> str:
        """Start CARLA simulation

        Args:
            scenario: Scenario name (optional)

        Returns:
            Simulation status
        """
        if self.simulation_running:
            logger.warning("Simulation already running")
            return "Simulation is already running"

        try:
            # Note: This is a placeholder. Actual CARLA startup would require
            # proper path configuration and environment setup
            logger.info(f"Starting CARLA simulation with scenario: {scenario}")
            self.simulation_running = True
            self.start_time = time.time()
            return f"CARLA simulation started with scenario: {scenario}"
        except Exception as e:
            logger.error(f"Failed to start simulation: {e}")
            return f"Failed to start simulation: {e}"

    def stop_carla_simulation(self) -> str:
        """Stop CARLA simulation

        Returns:
            Stop status
        """
        if not self.simulation_running:
            logger.warning("No simulation is running")
            return "No simulation is running"

        try:
            logger.info("Stopping CARLA simulation")
            if self.simulation_process:
                self.simulation_process.terminate()
                self.simulation_process.wait(timeout=5)
                self.simulation_process = None

            self.simulation_running = False
            runtime = time.time() - self.start_time if self.start_time else 0
            self.start_time = None
            logger.info(f"Simulation stopped after {runtime:.2f} seconds")
            return f"CARLA simulation stopped (runtime: {runtime:.2f}s)"
        except Exception as e:
            logger.error(f"Failed to stop simulation: {e}")
            return f"Failed to stop simulation: {e}"

    def get_simulation_status(self) -> str:
        """Get simulation status

        Returns:
            Running status, timestamp
        """
        if self.simulation_running:
            runtime = time.time() - self.start_time if self.start_time else 0
            logger.info(f"Simulation running for {runtime:.2f} seconds")
            return f"Simulation Status: RUNNING (runtime: {runtime:.2f}s)"
        else:
            logger.info("Simulation not running")
            return "Simulation Status: NOT RUNNING"
