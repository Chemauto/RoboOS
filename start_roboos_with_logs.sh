#!/bin/bash

# RoboOS 4进程启动脚本 (带实时日志)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/carla_roboos_logs"
TAIL_PID=""

# 捕获Ctrl+C信号 (必须在脚本开头定义)
trap cleanup INT TERM

cleanup() {
    echo ""
    echo "=== 正在停止RoboOS系统 ==="

    # 停止tail进程
    if [ -n "$TAIL_PID" ] && kill -0 "$TAIL_PID" 2>/dev/null; then
        kill "$TAIL_PID" 2>/dev/null
    fi

    # 停止RoboOS进程
    if [ -f "$LOG_DIR/pids_four.txt" ]; then
        PIDS=$(cat "$LOG_DIR/pids_four.txt")
        echo "停止进程: $PIDS"
        kill $PIDS 2>/dev/null
        sleep 2
        # 强制终止残留进程
        kill -9 $PIDS 2>/dev/null
    fi

    echo "✓ 所有进程已停止"
    exit 0
}

# 创建日志目录
mkdir -p "$LOG_DIR"

# 获取conda路径
CONDA_BASE=$(conda info --base)

echo "Starting 4-process RoboOS system..."
echo "Logs will be saved to: $LOG_DIR"
echo ""

# 启动所有进程
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
  python "$SCRIPT_DIR/master/run.py"
) > "$LOG_DIR/terminal2_roboos_master.log" 2>&1 &
PID2=$!

echo "[3/4] Starting RoboOS slaver..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  python "$SCRIPT_DIR/slaver/run.py"
) > "$LOG_DIR/terminal3_roboos_slaver.log" 2>&1 &
PID3=$!

echo "[4/4] Starting RoboOS deploy..."
(
  unset all_proxy; unset ALL_PROXY
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate roboOS
  python "$SCRIPT_DIR/deploy/run.py"
) > "$LOG_DIR/terminal4_roboos_deploy.log" 2>&1 &
PID4=$!

echo "$PID1 $PID2 $PID3 $PID4" > "$LOG_DIR/pids_four.txt"

echo ""
echo "All processes started! PIDs: $PID1 $PID2 $PID3 $PID4"
echo ""
echo "等待进程初始化..."
sleep 3

# 验证进程是否成功启动
echo ""
echo "=== 进程状态检查 ==="
FAILED=0
for pid in $PID1 $PID2 $PID3 $PID4; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "⚠️  进程 $pid 启动失败"
        FAILED=1
    fi
done

if [ $FAILED -eq 0 ]; then
    echo "✓ 所有进程运行正常"
else
    echo ""
    echo "部分进程启动失败,请检查日志"
fi

echo ""
echo "=== 实时日志输出 (按 Ctrl+C 停止) ==="
echo ""

# 使用tail -f显示所有日志,并保存PID以便cleanup时终止
tail -f "$LOG_DIR/terminal1_roboos_server.log" \
        "$LOG_DIR/terminal2_roboos_master.log" \
        "$LOG_DIR/terminal3_roboos_slaver.log" \
        "$LOG_DIR/terminal4_roboos_deploy.log" &
TAIL_PID=$!

# 等待tail进程
wait $TAIL_PID
