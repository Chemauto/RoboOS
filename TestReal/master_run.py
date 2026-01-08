import json

import psutil
import os
import sys

# Add project root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from flask import Flask, jsonify, request
from flask_cors import CORS
from TestReal.master_agent import NewGlobalAgent
from flask_socketio import SocketIO

app = Flask(__name__, static_folder="assets")
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*")


master_agent = NewGlobalAgent(config_path=os.path.join(os.path.dirname(__file__), 'master_config.yaml'))


def send_text_to_forntend(text):
    socketio.emit("text_update", {"data": text}, namespace="/")


@app.route("/system_status", methods=["GET"])
def system_status():
    """
    Get the system status.

    Returns:
        JSON response with system status
    """
    cpu_load = psutil.cpu_percent(interval=1)

    memory = psutil.virtual_memory()
    memory_usage = memory.percent

    return jsonify(
        {
            "cpu_load": round(cpu_load, 1),
            "memory_usage": round(memory_usage, 1),
        }
    )


@app.route("/robot_status", methods=["GET"])
def robot_status():
    """
    Get the status of all robots.

    Returns:
        JSON response with robot status
    """
    try:
        registered_robots = master_agent.collaborator.read_all_agents_info()
        registered_robots_status = []
        for robot_name, robot_info in registered_robots.items():
            registered_robots_status.append(
                {
                    "robot_name": robot_name,
                    "robot_state": json.loads(robot_info).get("robot_state"),
                }
            )
        return jsonify(registered_robots_status), 200
    except Exception as e:
        return jsonify({"error": "Internal server error", "details": str(e)}), 500


@app.route("/publish_task", methods=["POST", "GET"])
def publish_task():
    """
    Publish a task to the Redis channel.

    Request JSON format:
    {
        "task": "task_content"  # The task to be published
        "refresh": "true" # Boolean value, default is true, indicating whether to refresh the cached robot memory
    }

    Returns:
        JSON response with status or error message
    """
    if request.method == "GET":
        return jsonify({"statis": "success"}), 200
    try:
        data = request.get_json()
        if not data or "task" not in data:
            return jsonify({"error": "Invalid request - 'task' field required"}), 400
        task_content = data.get("task")
        if not task_content:
            return jsonify({"error": "Invalid request - 'task' field required"}), 400

        # The frontend might send a string or a list with one string. Handle both.
        if isinstance(task_content, list):
            if len(task_content) == 0:
                return jsonify({"error": "Invalid request - 'task' list is empty"}), 400
            task_content = task_content[0]

        if not isinstance(task_content, str):
            return jsonify({"error": "Invalid task format - must be a string"}), 400
            
        task_id = data.get("task_id")
        refresh = data.get("refresh", False)

        subtask_list = master_agent.publish_global_task(
            task_content, refresh, task_id
        )

        # Check if the agent returned a failure message
        if isinstance(subtask_list, dict) and subtask_list.get("status") == "error":
            return jsonify({
                "status": "error",
                "message": subtask_list.get("message", "Task decomposition failed.")
            }), 400

        return (
            jsonify(
                {
                    "status": "success",
                    "message": "Task published successfully",
                    "data": subtask_list,
                }
            ),
            200,
        )

    except Exception as e:
        return jsonify({"error": "Internal server error", "details": str(e)}), 500


if __name__ == "__main__":
    # Run the Flask app
    app.run(host="0.0.0.0", port=5000)
