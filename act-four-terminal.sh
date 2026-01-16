#!/bin/bash

# RoboOS 4进程启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/carla_roboos_logs"

# 捕获Ctrl+C信号 (必须在脚本开头定义)
trap cleanup INT TERM

cleanup() {
    echo ""
    echo "=== 正在停止RoboOS系统 ==="

    if [ -f "$LOG_DIR/pids_four.txt" ]; then
        PIDS=$(cat "$LOG_DIR/pids_four.txt")
        echo "停止进程: $PIDS"
        kill $PIDS 2>/dev/null
        sleep 2
        # 强制停止残留进程
        kill -9 $PIDS 2>/dev/null
    fi

    echo "✓ 所有进程已停止"
    exit 0
}

# 创建日志目录
mkdir -p "$LOG_DIR"

# 获取conda路径
CONDA_BASE=$(conda info --base)

echo "Starting 4-terminal RoboOS system..."
echo "Logs will be saved to: $LOG_DIR"

# 终端1: RoboOS server
echo "[1/4] Starting RoboOS server..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  bash "$SCRIPT_DIR/Modeldeploy/start_server.sh"
) > "$LOG_DIR/terminal1_roboos_server.log" 2>&1 &
PID1=$!

echo "[2/4] Starting RoboOS master..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  cd "$SCRIPT_DIR/master"
  python run.py
) > "$LOG_DIR/terminal2_roboos_master.log" 2>&1 &
PID2=$!

echo "[3/4] Starting RoboOS slaver..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  cd "$SCRIPT_DIR/slaver"
  python run.py
) > "$LOG_DIR/terminal3_roboos_slaver.log" 2>&1 &
PID3=$!

echo "[4/4] Starting RoboOS deploy..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  cd "$SCRIPT_DIR/deploy"
  python run.py
) > "$LOG_DIR/terminal4_roboos_deploy.log" 2>&1 &
PID4=$!

echo "$PID1 $PID2 $PID3 $PID4" > "$LOG_DIR/pids_four.txt"

echo ""
echo "All processes started! PIDs: $PID1 $PID2 $PID3 $PID4"
echo "Logs: $LOG_DIR/"
echo ""
echo "按 Ctrl+C 停止所有进程..."
echo ""

# 保持脚本运行,等待Ctrl+C
while true; do
    sleep 1
done
