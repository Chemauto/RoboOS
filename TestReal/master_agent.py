# /home/dora/RoboOs/RoboOS/TestReal/master_agent.py
# This is a copy of the original agent, modified to use the NewGlobalTaskPlanner
import asyncio
import json
import logging
import os
import re
import threading
import uuid
from collections import defaultdict
from typing import Dict

import yaml
# 重要：从 TestReal 目录导入新的 Planner
from master_planner import NewGlobalTaskPlanner
from flag_scale.flagscale.agent.collaboration import Collaborator

class NewGlobalAgent:
    def __init__(self, config_path="config.yaml"):
        self._init_config(config_path)
        self._init_logger(self.config["logger"])
        self.collaborator = Collaborator.from_config(self.config["collaborator"])
        # 使用新的 Planner
        self.planner = NewGlobalTaskPlanner(self.config)
        self.listening_robots = set()

        self.logger.info(f"Configuration loaded from {config_path} ...")
        self.logger.info(f"New Master Configuration:\n{self.config}")

        self._init_scene(self.config.get("profile", {}))
        self._start_listener()

    # _init_logger, _init_config, _init_scene, _handle_register, _handle_result,
    # _group_tasks_by_order, _start_listener, reasoning_and_subtasks_is_right,
    # and _dispath_subtasks_async methods are identical to the original agent.py
    # For brevity, we will paste them here without modification comments.

    def _init_logger(self, logger_config):
        self.logger = logging.getLogger(logger_config["master_logger_name"])
        logger_file = logger_config["master_logger_file"]
        os.makedirs(os.path.dirname(logger_file), exist_ok=True)
        # Avoid adding handlers multiple times
        if not self.logger.handlers:
            file_handler = logging.FileHandler(logger_file)
            log_level = getattr(logging, logger_config.get("master_logger_level", "INFO").upper(), logging.INFO)
            file_handler.setLevel(log_level)
            formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

    def _init_config(self, config_path="config.yaml"):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

    def _init_scene(self, scene_config):
        path = scene_config.get("path")
        if path and os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                self.scene = yaml.safe_load(f)
            scenes = self.scene.get("scene", [])
            for scene_info in scenes:
                scene_name = scene_info.pop("name", None)
                if scene_name:
                    self.collaborator.record_environment(scene_name, json.dumps(scene_info))
        else:
            self.logger.warning(f"Scene config file not found at {path}. Proceeding without scene info.")
            self.scene = {}

    def _handle_register(self, robot_name: Dict) -> None:
        if robot_name in self.listening_robots:
            return
        robot_info = self.collaborator.read_agent_info(robot_name)
        self.logger.info(f"AGENT_REGISTRATION: {robot_name} \n {json.dumps(robot_info)}")
        channel_r2b = f"{robot_name}_to_RoboOS"
        threading.Thread(
            target=lambda: self.collaborator.listen(channel_r2b, self._handle_result),
            daemon=True,
            name=channel_r2b,
        ).start()
        self.listening_robots.add(robot_name)
        self.logger.info(f"RoboOS has listened to [{robot_name}] by channel [{channel_r2b}]")

    def _handle_result(self, data: str):
        data = json.loads(data)
        robot_name = data.get("robot_name")
        subtask_handle = data.get("subtask_handle")
        subtask_result = data.get("subtask_result")
        if robot_name and subtask_handle and subtask_result:
            self.logger.info(f"================ Received result from {robot_name} ================")
            self.logger.info(f"Subtask: {subtask_handle}\nResult: {subtask_result}")
            self.logger.info("====================================================================")
            self.collaborator.update_agent_busy(robot_name, False)
        else:
            self.logger.warning("[WARNING] Received incomplete result data")

    def _extract_json(self, input_string: str) -> Dict:
        if not isinstance(input_string, str):
            self.logger.warning(f"[_extract_json] received non-string input: {type(input_string)}")
            return None
        try:
            # First, try to load the whole string as JSON
            return json.loads(input_string)
        except json.JSONDecodeError:
            # If that fails, look for markdown-style JSON
            json_match = re.search(r"```json\s*\n(.*?)\n\s*```", input_string, flags=re.DOTALL)
            if json_match:
                json_str = json_match.group(1).strip()
                try:
                    return json.loads(json_str)
                except json.JSONDecodeError as e:
                    self.logger.warning(f"Failed to parse JSON from markdown: {e}")
            self.logger.warning("Could not parse JSON from string content.")
            return None

    def _group_tasks_by_order(self, tasks):
        grouped = defaultdict(list)
        for task in tasks:
            grouped[int(task.get("subtask_order", 0))].append(task)
        return dict(sorted(grouped.items()))

    def _start_listener(self):
        threading.Thread(
            target=lambda: self.collaborator.listen("AGENT_REGISTRATION", self._handle_register),
            daemon=True,
        ).start()
        self.logger.info("Started listening for robot registrations...")

    def reasoning_and_subtasks_is_right(self, reasoning_and_subtasks: dict) -> bool:
        if not isinstance(reasoning_and_subtasks, dict) or "subtask_list" not in reasoning_and_subtasks:
            return False
        try:
            worker_list = {
                subtask["robot_name"]
                for subtask in reasoning_and_subtasks["subtask_list"]
                if isinstance(subtask, dict) and "robot_name" in subtask
            }
            if not worker_list: # Check if any robot was assigned
                return False
            robots_list = set(self.collaborator.read_all_agents_name())
            return worker_list.issubset(robots_list)
        except (TypeError, KeyError):
            return False

    def publish_global_task(self, task: str, refresh: bool, task_id: str) -> Dict:
        self.logger.info(f"New Master Agent publishing global task: {task}")
        response = self.planner.forward(task)
        reasoning_and_subtasks = self._extract_json(response)
        
        if not self.reasoning_and_subtasks_is_right(reasoning_and_subtasks):
            self.logger.error(f"Task decomposition failed validation: {reasoning_and_subtasks}")
            # 返回符合前端期望的标准错误格式
            return {"status": "error", "message": "Task decomposition failed validation or LLM planner returned invalid data."}

        self.logger.info(f"Received reasoning and subtasks:\n{json.dumps(reasoning_and_subtasks, indent=2)}")
        subtask_list = reasoning_and_subtasks.get("subtask_list", [])
        grouped_tasks = self._group_tasks_by_order(subtask_list)
        task_id = task_id or str(uuid.uuid4()).replace("-", "")
        threading.Thread(
            target=asyncio.run,
            args=(self._dispath_subtasks_async(task, task_id, grouped_tasks, refresh),),
            daemon=True,
        ).start()
        return reasoning_and_subtasks

    async def _dispath_subtasks_async(self, task: str, task_id: str, grouped_tasks: Dict, refresh: bool):
        order_flag = "false" if len(grouped_tasks.keys()) == 1 else "true"
        for order, group_task in grouped_tasks.items():
            self.logger.info(f"Sending task group {order}:\n{group_task}")
            working_robots = [tasks.get("robot_name") for tasks in group_task]
            for tasks in group_task:
                robot_name = tasks.get("robot_name")
                if not robot_name: continue
                subtask_data = {"task_id": task_id, "task": tasks["subtask"], "order": order_flag}
                if refresh:
                    self.collaborator.clear_agent_status(robot_name)
                self.collaborator.send(f"roboos_to_{robot_name}", json.dumps(subtask_data))
                self.collaborator.update_agent_busy(robot_name, True)
            self.collaborator.wait_agents_free(working_robots)
        self.logger.info(f"Task_id ({task_id}) [{task}] has been sent to all agents.")
