# CARLA显存占用分析

## 当前配置 (总计约4.4GB)

### 1. 传感器配置 (dora.py:66-73)
```python
- GNSS (GPS): ~10MB
- IMU: ~10MB
- LIDAR: ~1.5-2GB ⚠️ 主要显存消耗
  - range: 5m
  - channels: 16
  - points_per_second: 50000
  - rotation_frequency: 10Hz
```

### 2. 场景渲染 (CarlaUE4.sh:5)
```bash
- 分辨率: 400x300
- 质量: Low
- FPS: 15
- 窗口模式
估计: ~1.5-2GB
```

### 3. Leaderboard框架
```
- 场景加载器
- 车辆模型和物理引擎
- 路径规划数据
- 多场景资源预加载
估计: ~500-800MB
```

---

## 优化方案 (针对简单车辆控制测试)

### 方案A: 最小传感器配置 (推荐)
**预期节省: 1.5-2GB**

移除LIDAR,只保留GNSS+IMU:
```python
def sensors(self):
    return [
        {"type": "sensor.other.gnss", "id": "GPS", "x": 0, "y": 0, "z": 1.60},
        {"type": "sensor.other.imu", "id": "IMU", "x": 0, "y": 0, "z": 1.60}
    ]
```

### 方案B: 轻量级LIDAR (如果需要障碍物检测)
**预期节省: 1-1.5GB**

大幅降低LIDAR参数:
```python
{"type": "sensor.lidar.ray_cast", "id": "LIDAR",
 "range": 10,              # 从5增加到10米
 "channels": 8,            # 从16减少到8
 "points_per_second": 10000,  # 从50000减少到10000
 "rotation_frequency": 5}  # 从10Hz减少到5Hz
```

### 方案C: 无渲染模式
**预期节省: 1.5-2GB**

修改CarlaUE4.sh添加:
```bash
-RenderOffScreen
```
⚠️ 注意: 根据RELEASE.md,此模式可能导致车辆不生成

### 方案D: 跳过Leaderboard框架
**预期节省: 500-800MB**

直接使用CARLA Python API,不通过Leaderboard:
```python
# 简单的车辆控制脚本
import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
# 直接spawn车辆和控制
```

---

## 推荐组合方案

### 最小配置 (预期总显存: 1.5-2GB)
1. ✅ 移除LIDAR (方案A)
2. ✅ 跳过Leaderboard (方案D)
3. ✅ 保持当前渲染设置

**预期结果:**
- CARLA: 1.5-2GB
- VLLM: 9.6GB
- 空闲: 4.8-5GB ✓ 足够spawn车辆

### 中等配置 (预期总显存: 2.5-3GB)
1. ✅ 轻量级LIDAR (方案B)
2. ✅ 跳过Leaderboard (方案D)
3. ✅ 保持当前渲染设置

---

## 实施步骤

### 快速测试 (方案A+D)
1. 创建简单的Python控制脚本 (不使用Leaderboard)
2. 只启动CARLA服务器
3. 直接通过Python API控制车辆

### 修改现有系统 (方案A)
1. 编辑 `/home/dora/RoboOS/Vehicle/CARLA_Leaderboard_20/leaderboard/leaderboard/autoagents/dora.py`
2. 修改 `sensors()` 方法,移除LIDAR
3. 注释掉所有LIDAR相关代码

---

## 显存监控命令
```bash
# 实时监控
watch -n 1 'nvidia-smi --query-gpu=memory.used,memory.free --format=csv,noheader,nounits'

# 详细进程显存
nvidia-smi pmon -c 1
```
