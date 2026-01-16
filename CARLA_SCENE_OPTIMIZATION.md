# CARLA场景渲染优化方案

## 当前配置分析

**当前地图**: Town12 (复杂城市场景)
- 大量建筑物、交通灯、路标
- 复杂的道路网络
- 高多边形数量
- **预估显存**: 2-2.5GB

**当前渲染设置** (CarlaUE4.sh):
```bash
-quality-level=Low
-windowed
-ResX=400 -ResY=300
-benchmark -fps=15
```

---

## 优化方案

### 方案1: 使用最简单地图 (推荐)
**预期节省: 800MB-1.2GB**

切换到Town01 (最简单的小镇):
- 基础道路网络
- 最少的建筑物
- 适合车辆控制测试

**实施方法A**: 修改routes配置
```xml
<!-- 编辑 routes_devtest.xml -->
<route id="0" town="Town01">  <!-- 从Town12改为Town01 -->
```

**实施方法B**: 直接启动指定地图
```bash
# 在CarlaUE4.sh中添加地图参数
./CarlaUE4.sh Town01
```

### 方案2: 使用优化版地图
**预期节省: 300-500MB**

使用Town01_Opt (预优化版本):
```bash
./CarlaUE4.sh Town01_Opt
```

### 方案3: 进一步降低渲染质量
**预期节省: 200-400MB**

修改CarlaUE4.sh:
```bash
-quality-level=Low \
-ResX=320 -ResY=240 \     # 从400x300降至320x240
-benchmark -fps=10 \       # 从15fps降至10fps
-no-rendering-shadows \    # 禁用阴影
-no-rendering-reflections  # 禁用反射
```

### 方案4: 空地图模式 (最激进)
**预期节省: 1.5-2GB**

使用空白地图,只加载必要的道路:
```python
# Python API方式
client = carla.Client('localhost', 2000)
world = client.load_world('Town01', carla.MapLayer.NONE)  # 不加载建筑物
```

---

## 推荐组合方案

### 最小显存配置 (预期CARLA总显存: 1-1.5GB)

1. ✅ 使用Town01_Opt地图
2. ✅ 移除LIDAR传感器
3. ✅ 降低分辨率到320x240
4. ✅ 禁用阴影和反射
5. ✅ 跳过Leaderboard框架

**预期结果:**
```
CARLA场景:    800MB  (Town01_Opt + 低质量)
GNSS+IMU:      20MB
系统开销:     200MB
─────────────────────
CARLA总计:   ~1GB

VLLM:        9.6GB
空闲显存:    5.4GB ✓✓✓ 充足
```

---

## 实施步骤

### 快速测试 (不修改现有配置)

创建简单的Python脚本直接控制:

```python
#!/usr/bin/env python3
import carla
import time

# 连接CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 加载最简单的地图
world = client.load_world('Town01_Opt')

# 获取蓝图库
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

# 在起点spawn车辆
spawn_points = world.get_map().get_spawn_points()
vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

print(f"车辆已生成: {vehicle.id}")
print(f"位置: {vehicle.get_location()}")

# 简单控制循环
try:
    while True:
        # 这里可以接收UDP控制指令
        control = carla.VehicleControl()
        control.throttle = 0.3
        vehicle.apply_control(control)
        time.sleep(0.1)
except KeyboardInterrupt:
    vehicle.destroy()
    print("车辆已销毁")
```

### 修改现有配置

1. **修改地图** (routes_devtest.xml:2):
```bash
sed -i 's/town="Town12"/town="Town01_Opt"/' /home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/leaderboard/data/routes_devtest.xml
```

2. **优化渲染** (CarlaUE4.sh:5):
```bash
# 添加更多优化参数
-quality-level=Low -windowed -ResX=320 -ResY=240 -benchmark -fps=10
```

---

## 地图复杂度对比

| 地图 | 建筑物 | 道路复杂度 | 预估显存 |
|------|--------|-----------|---------|
| Town01 | 少 | 简单 | 800MB |
| Town01_Opt | 少 | 简单 | 600MB |
| Town02 | 中 | 中等 | 1.2GB |
| Town03 | 多 | 复杂 | 1.8GB |
| Town12 | 很多 | 很复杂 | 2.5GB |

---

## 验证方法

启动CARLA后检查显存:
```bash
# 启动CARLA (Town01_Opt)
cd /home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20
./CarlaUE4.sh Town01_Opt &

# 等待10秒加载
sleep 10

# 检查显存
nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits
```
