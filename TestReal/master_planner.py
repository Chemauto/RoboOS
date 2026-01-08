# /home/dora/RoboOs/RoboOS/TestReal/master_planner.py
import json
from typing import Any, Dict, Union

# 导入我们项目自定义的 Model 类
from TestReal.agents.models import OpenAIServerModel, AzureOpenAIServerModel
from master_prompts import MASTER_PLANNING_PLANNING
from flag_scale.flagscale.agent.collaboration import Collaborator

class NewGlobalTaskPlanner:
    """A tool planner that understands when to delegate vs. decompose."""

    def __init__(
        self,
        config: Union[Dict, str] = None,
    ) -> None:
        self.config = config # 保存配置以供后续使用
        self.collaborator = Collaborator.from_config(config["collaborator"])
        self.profiling = config.get("profiling", False) # 提前初始化 profiling

        self.global_model: Any
        self.model_name: str
        self.global_model, self.model_name = self._get_model_info_from_config()

    def _get_model_info_from_config(self) -> tuple:
        """Get the model info from config, using our custom model classes."""
        model_config = self.config["model"]
        candidate = model_config["model_dict"]
        
        model_client = OpenAIServerModel(
            api_key=candidate["cloud_api_key"],
            # 确保使用正确的 api_base 地址
            api_base=candidate["cloud_server"],
            model_id=candidate["cloud_model"],
            # 这些参数在 Master 端可能不直接使用，但为了保持一致性而包含
            support_tool_calls=self.config.get("tool", {}).get("support_tool_calls", False),
            profiling=self.profiling,
        )
        model_name = model_config["model_select"]
        return model_client, model_name

    def display_profiling_info(self, description: str, message: any):
        if self.profiling:
            print(f" [New Master Planner] {description}:\n{message}\n")

    def forward(self, task: str) -> str:
        """
        Decides whether to delegate the task or break it down.
        """
        all_robots_info = self.collaborator.read_all_agents_info()
        all_environments_info = self.collaborator.read_environment(name=None)
        
        robot_tools_for_llm = []
        if all_robots_info:
            first_robot_name = next(iter(all_robots_info))
            robot_tools_for_llm = all_robots_info[first_robot_name].get("robot_tool", [])

        scene_and_robot_info = f"Scene Info: {json.dumps(all_environments_info, indent=2)}\n"
        scene_and_robot_info += f"Robot Tools Info: {json.dumps(robot_tools_for_llm, indent=2)}"

        content = MASTER_PLANNING_PLANNING.format(
            scene_info=scene_and_robot_info,
            task=task,
        )

        messages = [{"role": "user", "content": content}]
        
        self.display_profiling_info("Sending to Master LLM", messages)
        
        # 使用我们自定义的 generate 方法
        response = self.global_model.generate(
            messages=messages,
            temperature=0.1,
            # vllm 可能不支持 response_format，我们依赖 prompt 来确保 JSON 输出
            # response_format={"type": "json_object"}, 
        )
        
        # 我们的 generate 方法返回一个包含 choices 的字典
        response_content = response['choices'][0]['message']['content']
        self.display_profiling_info("Raw Response from Master LLM", response_content)
        
        return response_content