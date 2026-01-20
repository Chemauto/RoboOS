#!/usr/bin/env python
# coding=utf-8
import json
import time
from logging import getLogger
from typing import Any, Callable, Dict, List, Optional, Union

from slaver.agents.models import ChatMessage
from flag_scale.flagscale.agent.collaboration import Collaborator
from mcp import ClientSession
from rich.panel import Panel
from rich.text import Text
from slaver.tools.memory import ActionStep, AgentMemory, SceneMemory
from slaver.tools.monitoring import AgentLogger, LogLevel, Monitor

logger = getLogger(__name__)


class MultiStepAgent:
    """
    Agent class that solves the given task step by step, using the ReAct framework:
    While the objective is not reached, the agent will perform a cycle of action (given by the LLM) and observation (obtained from the environment).

    Args:
        tools (`list[Tool]`): [`Tool`]s that the agent can use.
        max_steps (`int`, default `20`): Maximum number of steps the agent can take to solve the task.
        verbosity_level (`LogLevel`, default `LogLevel.INFO`): Level of verbosity of the agent's logs.
        step_callbacks (`list[Callable]`, *optional*): Callbacks that will be called at each step.
    """

    def __init__(
        self,
        tools: List[Dict[str, str]],
        model: Callable[[List[Dict[str, str]]], ChatMessage],
        model_path: str,
        collaborator: Collaborator,
        tool_executor: ClientSession,
        robot_name: str,
        max_steps: int = 20,
        verbosity_level: LogLevel = LogLevel.INFO,
        step_callbacks: Optional[List[Callable]] = None,
        log_file: Optional[str] = None,
    ):
        self.tools = tools
        self.model = model
        self.model_path = model_path
        self.collaborator = collaborator
        self.robot_name = robot_name
        self.tool_executor = tool_executor
        self.max_steps = max_steps
        self.step_number = 0
        self.state = {}
        self.memory = AgentMemory()
        self.scene = SceneMemory(collaborator)
        self.logger = AgentLogger(level=verbosity_level, log_file=log_file)
        self.monitor = Monitor(self.model, self.logger)
        self.step_callbacks = step_callbacks if step_callbacks is not None else []
        self.step_callbacks.append(self.monitor.update_metrics)

    async def run(
        self,
        task: str,
        reset: bool = True,
        images: Optional[List[str]] = None,
        max_steps: Optional[int] = None,
    ):
        """
        Run the agent for the given task.

        Args:
            task (`str`): Task to perform.
            reset (`bool`): Whether to reset the conversation or keep it going from previous run.
            images (`list[str]`, *optional*): Paths to image(s).
            max_steps (`int`, *optional*): Maximum number of steps the agent can take to solve the task. if not provided, will use the agent's default value.

        Example:
        ```py
        from smolagents import CodeAgent
        agent = CodeAgent(tools=[])
        agent.run("What is the result of 2 power 3.7384?")
        ```
        """
        max_steps = max_steps or self.max_steps
        self.task = task

        if reset:
            self.memory.reset()
            self.step_number = 1

            # ========== 方案1: 任务级别的历史清空 ==========
            # 每个新任务开始时清空历史状态，避免历史污染
            # 这样可以确保每个任务都是独立执行的
            try:
                # 清空当前 robot 的历史状态
                self.collaborator.clear_agent_status(self.robot_name)
                logger.info(f"[History Management] Cleared history for new task: {self.robot_name}")
            except AttributeError:
                # 如果 collaborator 没有 clear_agent_status 方法，则尝试手动清空
                # 通过记录空状态来实现清空效果
                try:
                    self.collaborator.record_agent_status(self.robot_name, "")
                    logger.info(f"[History Management] Reset history by recording empty status: {self.robot_name}")
                except Exception as e:
                    logger.warning(f"[History Management] Failed to clear history: {e}")
            # ================================================

        self.logger.log_task(
            content=self.task.strip(),
            subtitle=f"{type(self.model).__name__} - {(self.model.model_id if hasattr(self.model, 'model_id') else '')}",
            level=LogLevel.INFO,
            title=self.name if hasattr(self, "name") else None,
        )

        while self.step_number <= max_steps:
            step_start_time = time.time()
            step = ActionStep(
                step_number=self.step_number,
                start_time=step_start_time,
                observations_images=images,
            )
            answer = await self.step(step)
            if answer == "final_answer":
                return "Mission accomplished"

            self.collaborator.record_agent_status(self.robot_name, answer)
            step.end_time = time.time()
            self.step_number += 1

        return "Maximum number of attempts reached, Mission not completed"

    def step(self) -> Optional[Any]:
        """To be implemented in children classes. Should return either None if the step is not final."""
        raise NotImplementedError


class ToolCallingAgent(MultiStepAgent):
    """
    This agent uses JSON-like tool calls, using method `model.get_tool_call` to leverage the LLM engine's tool calling capabilities.

    Args:
        tools (`list[Tool]`): [`Tool`]s that the agent can use.
        prompt_templates ([`~agents.PromptTemplates`], *optional*): Prompt templates.
        planning_interval (`int`, *optional*): Interval at which the agent will run a planning step.
        **kwargs: Additional keyword arguments.
    """

    def __init__(
        self,
        tools: List[Dict[str, str]],
        model: Callable[[List[Dict[str, str]]], ChatMessage],
        model_path: str,
        collaborator: Collaborator,
        robot_name: str,
        **kwargs,
    ):
        self.tool_call = []
        super().__init__(
            tools=tools,
            model=model,
            model_path=model_path,
            collaborator=collaborator,
            robot_name=robot_name,
            **kwargs,
        )

    async def _execute_tool_call(
        self, tool_name: str, tool_arguments: dict, memory_step: ActionStep
    ) -> Union[str, None]:
        self.logger.log(
            Panel(
                Text(f"Calling tool: '{tool_name}' with arguments: {tool_arguments}")
            ),
            level=LogLevel.INFO,
        )
        # Handle both string and dict formats for tool_arguments
        args = tool_arguments if isinstance(tool_arguments, dict) else json.loads(tool_arguments)
        observation = await self.tool_executor(tool_name, args)
        observation = observation.content[0].text
        self.logger.log(
            f"Observations: {observation.replace('[', '|')}",  # escape potential rich-tag-like components
            level=LogLevel.INFO,
        )

        # Construct memory input
        memory_input = {
            "tool_name": tool_name,
            "arguments": tool_arguments,
            "result": observation,
        }
        try:
            await self.memory_predict(memory_input)
        except Exception as e:
            print(f"[Scene Update Error] `{e}`")

        return observation

    async def memory_predict(self, memory_input: dict) -> str:
        """
        Use the model to predict the scene-level effect of the current tool execution.
        Possible effects: add_object, remove_object, move_object, position.
        """

        prompt = self.scene.get_action_type_prompt(memory_input)

        model_message: ChatMessage = self.model(
            task=prompt, current_status="", model_path=self.model_path
        )

        action_type = model_message.content.strip().lower()

        # Handle both string and dict formats for arguments
        args = memory_input["arguments"] if isinstance(memory_input["arguments"], dict) else json.loads(memory_input["arguments"])
        self.scene.apply_action(action_type, args)

    async def step(self, memory_step: ActionStep) -> Union[None, Any]:
        """
        Perform one step in the ReAct framework: the agent thinks, acts, and observes the result.
        Returns None if the step is not final.
        """
        self.logger.log_rule(f"Step {self.step_number}", level=LogLevel.INFO)

        # Add new step in logs
        current_status = self.collaborator.read_agent_status(self.robot_name)

        # ========== 方案2: 限制历史长度 ==========
        # 只保留最近 2 条历史记录，防止历史无限累积
        # 这是一个保险措施，即使方案1失败也能限制历史污染
        MAX_HISTORY_LENGTH = 2
        if isinstance(current_status, list) and len(current_status) > MAX_HISTORY_LENGTH:
            current_status = current_status[-MAX_HISTORY_LENGTH:]
            logger.info(f"[History Management] Trimmed history to last {MAX_HISTORY_LENGTH} items")
        # ==========================================

        model_message: ChatMessage = self.model(
            task=self.task,
            current_status=current_status,
            model_path=self.model_path,
            tools_to_call_from=self.tools,
            stop_sequences=["Observation:"],
        )
        memory_step.model_output_message = model_message
        self.logger.log_markdown(
            content=(
                model_message.content
                if model_message.content
                else str(model_message.raw)
            ),
            title="Output message of the LLM:",
            level=LogLevel.DEBUG,
        )
        if model_message.tool_calls:
            tool_call = model_message.tool_calls[0]
            tool_name = tool_call.function.name
            tool_arguments = tool_call.function.arguments
        else:
            # 尝试从content中解析工具调用(兼容某些模型)
            try:
                import json
                import re
                content = model_message.content.strip()

                # 尝试多种格式:
                # 1. 纯JSON: {"name": "...", "arguments": {...}}
                # 2. 括号包裹: ({"name": "...", "arguments": {...}})
                # 3. 带额外文本: ({"name": "..."}) <tool_call>

                # 提取JSON部分
                json_match = re.search(r'\{[^{}]*"name"[^{}]*"arguments"[^{}]*\{[^}]*\}[^}]*\}', content)
                if json_match:
                    json_str = json_match.group(0)
                    tool_data = json.loads(json_str)
                    if 'name' in tool_data and 'arguments' in tool_data:
                        tool_name = tool_data['name']
                        tool_arguments = tool_data['arguments']
                        self.logger.log_markdown(
                            content=f"Parsed tool call from content: {tool_name}",
                            title="Tool Call Parsing:",
                            level=LogLevel.DEBUG,
                        )
                    else:
                        return "final_answer"
                else:
                    return "final_answer"
            except (json.JSONDecodeError, AttributeError, Exception):
                return "final_answer"

        current_call = {"tool_name": tool_name, "tool_arguments": tool_arguments}

        if self.tool_call and self.tool_call[-1] == current_call:
            return "final_answer"
        else:
            self.tool_call.append(current_call)

        return await self._execute_tool_call(tool_name, tool_arguments, memory_step)
