#!/bin/bash

# 创建日志目录
LOG_DIR="/tmp/carla_roboos_logs"
mkdir -p $LOG_DIR

# 获取conda路径
CONDA_BASE=$(conda info --base)

echo "Starting 6-terminal CARLA + RoboOS simulation..."
echo "Logs will be saved to: $LOG_DIR"

# 终端1: CARLA仿真服务器
echo "[1/6] Starting CARLA server..."
/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/CarlaUE4.sh > $LOG_DIR/terminal1_carla_server.log 2>&1 &
PID1=$!
echo "  PID: $PID1"

# 等待CARLA服务器启动
sleep 5

# 终端2: Leaderboard测试
echo "[2/6] Starting Leaderboard test..."
(
  source $CONDA_BASE/etc/profile.d/conda.sh
  conda activate py37
  export PYTHONPATH="${PYTHONPATH}:/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/scenario_runner:/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/PythonAPI/carla"
  cd /home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/leaderboard
  bash test_run.sh
) > $LOG_DIR/terminal2_leaderboard.log 2>&1 &
PID2=$!
echo "  PID: $PID2"

# 终端3-6: RoboOS组件
echo "[3/6] Starting RoboOS server..."
(
  unset all_proxy; unset ALL_PROXY
  source $CONDA_BASE/etc/profile.d/conda.sh
  conda activate roboOS
  bash /home/dora/RoboOS/Modeldeploy/start_server.sh
) > $LOG_DIR/terminal3_roboos_server.log 2>&1 &
PID3=$!

echo "[4/6] Starting RoboOS master..."
(
  unset all_proxy; unset ALL_PROXY
  source $CONDA_BASE/etc/profile.d/conda.sh
  conda activate roboOS
  python /home/dora/RoboOS/master/run.py
) > $LOG_DIR/terminal4_roboos_master.log 2>&1 &
PID4=$!

echo "[5/6] Starting RoboOS slaver..."
(
  unset all_proxy; unset ALL_PROXY
  source $CONDA_BASE/etc/profile.d/conda.sh
  conda activate roboOS
  python /home/dora/RoboOS/slaver/run.py
) > $LOG_DIR/terminal5_roboos_slaver.log 2>&1 &
PID5=$!

echo "[6/6] Starting RoboOS deploy..."
(
  unset all_proxy; unset ALL_PROXY
  source $CONDA_BASE/etc/profile.d/conda.sh
  conda activate roboOS
  python /home/dora/RoboOS/deploy/run.py
) > $LOG_DIR/terminal6_roboos_deploy.log 2>&1 &
PID6=$!

echo "$PID1 $PID2 $PID3 $PID4 $PID5 $PID6" > $LOG_DIR/pids.txt

echo ""
echo "All processes started! PIDs: $PID1 $PID2 $PID3 $PID4 $PID5 $PID6"
echo "Logs: $LOG_DIR/"
echo "Stop all: kill \$(cat $LOG_DIR/pids.txt)"
