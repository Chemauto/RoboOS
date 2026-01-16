#!/bin/bash
# 静态CARLA车辆控制系统
# 不使用Leaderboard,场景不切换

CARLA_ROOT="/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20"
LOG_DIR="/tmp/carla_static"

mkdir -p "$LOG_DIR"

echo "=== 静态CARLA车辆控制系统 ==="
echo ""

# 启动CARLA服务器
echo "1. 启动CARLA服务器 (Town04_Opt)..."
cd "$CARLA_ROOT"
vblank_mode=0 __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb SDL_VIDEODRIVER=x11 \
bash CarlaUE4.sh > "$LOG_DIR/carla.log" 2>&1 &
CARLA_PID=$!
echo "   CARLA PID: $CARLA_PID"

# 等待CARLA就绪
echo ""
echo "2. 等待CARLA服务器就绪..."
for i in {1..30}; do
    if netstat -tln 2>/dev/null | grep -q ":2000 " || ss -tln 2>/dev/null | grep -q ":2000 "; then
        echo "   ✓ CARLA服务器已就绪"
        break
    fi
    sleep 1
done

# 启动车辆控制脚本
echo ""
echo "3. 启动车辆控制脚本..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate py37
export PYTHONPATH="$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg"

python /home/dora/RoboOS/simple_vehicle_control.py > "$LOG_DIR/vehicle.log" 2>&1 &
VEHICLE_PID=$!
echo "   车辆控制 PID: $VEHICLE_PID"

echo ""
echo "=== 系统启动完成 ==="
echo ""
echo "日志文件:"
echo "  CARLA:  $LOG_DIR/carla.log"
echo "  车辆:   $LOG_DIR/vehicle.log"
echo ""
echo "监控命令:"
echo "  tail -f $LOG_DIR/vehicle.log"
echo ""
echo "按 Ctrl+C 停止..."
echo ""

# 等待中断
trap "pkill -P $$; exit" INT TERM
wait
