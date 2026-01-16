#!/bin/bash

# CARLA + Leaderboard 启动脚本
# 创建时间: 2026-01-16
# 修复时间: 2026-01-16

CARLA_ROOT="/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20"
LOG_DIR="/tmp/carla_roboos_logs"

# 捕获Ctrl+C信号 (必须在脚本开头定义)
trap cleanup INT TERM

cleanup() {
    echo ""
    echo "=== 正在停止CARLA系统 ==="

    # 停止Leaderboard
    if pgrep -f "leaderboard_evaluator" > /dev/null; then
        echo "停止Leaderboard..."
        pkill -f "leaderboard_evaluator"
        sleep 2
        # 强制终止残留进程
        if pgrep -f "leaderboard_evaluator" > /dev/null; then
            echo "  强制停止..."
            pkill -9 -f "leaderboard_evaluator"
        fi
    fi

    # 停止CARLA
    if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null; then
        echo "停止CARLA服务器..."
        pkill -f "CarlaUE4-Linux-Shipping"
        sleep 2
        # 强制终止残留进程
        if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null; then
            echo "  强制停止..."
            pkill -9 -f "CarlaUE4-Linux-Shipping"
        fi
    fi

    echo "✓ 所有进程已停止"
    exit 0
}

# 创建日志目录
mkdir -p "$LOG_DIR"

echo "=== CARLA系统启动脚本 ==="
echo ""

# 检查是否已有CARLA进程在运行
CARLA_RUNNING=false
if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null; then
    echo "⚠️  CARLA服务器已在运行"
    read -p "是否要重启? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "停止现有CARLA进程..."
        pkill -f "CarlaUE4-Linux-Shipping" || true
        sleep 3
    else
        echo "保持现有CARLA进程"
        CARLA_RUNNING=true
    fi
fi

# 检查是否已有Leaderboard进程在运行
LEADERBOARD_RUNNING=false
if pgrep -f "leaderboard_evaluator" > /dev/null; then
    echo "⚠️  Leaderboard已在运行"
    read -p "是否要重启? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "停止现有Leaderboard进程..."
        pkill -f "leaderboard_evaluator" || true
        sleep 2
    else
        echo "保持现有Leaderboard进程"
        LEADERBOARD_RUNNING=true
    fi
fi

# 如果两个都在运行且用户选择保持,直接退出
if [ "$CARLA_RUNNING" = true ] && [ "$LEADERBOARD_RUNNING" = true ]; then
    echo ""
    echo "✓ 系统已在运行,无需启动"
    exit 0
fi

# 启动CARLA服务器(如果需要)
if [ "$CARLA_RUNNING" = false ]; then
    echo ""
    echo "1. 启动CARLA服务器..."
    cd "$CARLA_ROOT"
    vblank_mode=0 __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb SDL_VIDEODRIVER=x11 bash CarlaUE4.sh > "$LOG_DIR/carla_server.log" 2>&1 &
    CARLA_PID=$!
    echo "   CARLA服务器已启动 (PID: $CARLA_PID)"
    echo "   日志: $LOG_DIR/carla_server.log"

    # 等待CARLA服务器就绪
    echo ""
    echo "2. 等待CARLA服务器就绪..."
    for i in {1..30}; do
        if netstat -tln 2>/dev/null | grep -q ":2000 " || ss -tln 2>/dev/null | grep -q ":2000 "; then
            echo "   ✓ CARLA服务器已就绪 (端口2000监听中)"
            break
        fi
        if [ $i -eq 30 ]; then
            echo "   ⚠️  CARLA服务器可能未成功启动 (端口2000未监听)"
            echo "   请检查日志: tail -f $LOG_DIR/carla_server.log"
        fi
        sleep 1
    done
else
    echo ""
    echo "1. CARLA服务器已在运行,跳过启动"
fi

# 启动Leaderboard(如果需要)
if [ "$LEADERBOARD_RUNNING" = false ]; then
    echo ""
    echo "3. 启动Leaderboard (dora.py agent)..."
    cd "$CARLA_ROOT/leaderboard"

    # 检查conda环境
    if ! conda env list | grep -q "py37"; then
        echo "   ✗ conda环境py37不存在!"
        echo "   请运行: conda env list 查看可用环境"
        exit 1
    fi

    # 设置环境变量并启动
    (
        source "$(conda info --base)/etc/profile.d/conda.sh"
        conda activate py37
        export CARLA_ROOT="$CARLA_ROOT"
        export PYTHONPATH="$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:${CARLA_ROOT}/scenario_runner:${CARLA_ROOT}/leaderboard:${CARLA_ROOT}/PythonAPI/carla"
        bash test_run.sh
    ) > "$LOG_DIR/leaderboard.log" 2>&1 &
    LEADERBOARD_PID=$!
    echo "   Leaderboard已启动 (PID: $LEADERBOARD_PID)"
    echo "   日志: $LOG_DIR/leaderboard.log"

    # 等待Leaderboard初始化
    echo ""
    echo "4. 等待Leaderboard初始化..."
    for i in {1..20}; do
        if lsof -i UDP:23456 > /dev/null 2>&1; then
            echo "   ✓ Leaderboard已就绪 (UDP端口23456监听中)"
            break
        fi
        if [ $i -eq 20 ]; then
            echo "   ⚠️  UDP端口23456未监听,可能还在初始化"
            echo "   请检查日志: tail -f $LOG_DIR/leaderboard.log"
        fi
        sleep 1
    done
else
    echo ""
    echo "3. Leaderboard已在运行,跳过启动"
fi

# 检查进程状态
echo ""
echo "=== 系统状态 ==="

# 检查CARLA
if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null; then
    CARLA_PID=$(pgrep -f "CarlaUE4-Linux-Shipping" | head -1)
    echo "✓ CARLA服务器运行中 (PID: $CARLA_PID)"
    if netstat -tln 2>/dev/null | grep -q ":2000 " || ss -tln 2>/dev/null | grep -q ":2000 "; then
        echo "  └─ 端口2000正在监听"
    fi
else
    echo "✗ CARLA服务器未运行"
fi

# 检查Leaderboard
if pgrep -f "leaderboard_evaluator" > /dev/null; then
    LB_PID=$(pgrep -f "leaderboard_evaluator")
    echo "✓ Leaderboard运行中 (PID: $LB_PID)"
else
    echo "✗ Leaderboard未运行"
fi

# 检查UDP端口
if lsof -i UDP:23456 > /dev/null 2>&1; then
    echo "✓ UDP端口23456正在监听"
else
    echo "⚠️  UDP端口23456未监听"
fi

echo ""
echo "=== 启动完成 ==="
echo ""
echo "查看日志:"
echo "  CARLA:       tail -f $LOG_DIR/carla_server.log"
echo "  Leaderboard: tail -f $LOG_DIR/leaderboard.log"
echo ""
echo "测试控制:"
echo "  python3 /home/dora/RoboOS/test_vehicle_control.py"
echo ""
echo "按 Ctrl+C 停止所有进程..."
echo ""

# 保持脚本运行,等待Ctrl+C
while true; do
    sleep 1
done
