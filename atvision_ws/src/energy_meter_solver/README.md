# Energy Meter Solver (能量机关求解器)

ROS2 能量机关检测与预测功能包，用于 RoboMaster 机器人的大小能量机关击打。

## 架构概览

```
rune_detector/marker (MarkerArray)
    ↓
┌────────────────────────────────────────┐
│         EnergyMeterSolverNode          │
│  ┌──────────────────────────────────┐  │
│  │     EnergyMeterTracker (ISRCKF)  │  │ ← 状态估计与滤波
│  └──────────────────────────────────┘  │
│  ┌──────────────────────────────────┐  │
│  │     AngleFitter (异步拟合)        │  │ ← 运动模型参数拟合
│  └──────────────────────────────────┘  │
└────────────────────────────────────────┘
    ↓
energy_meter_solver/state (EnergyMeterState)
```

## 核心模块

### 1. ISRCKF (迭代平方根容积卡尔曼滤波器)

`include/energy_meter_solver/isrckf.hpp`

- 2N 容积点（权重全正），避免 UKF 负权重问题
- Gauss-Newton / MAP 迭代量测更新
- LDLT + LLT 自适应 Cholesky 分解，保证数值稳定性
- QR 分解协方差合成

状态向量 (5维)：
```
[XC, YC, ZC, ROLL, V_ROLL]
 R中心xyz   0号靶角度  角速度
```

### 2. EnergyMeterTracker

`include/energy_meter_solver/energy_meter_tracker.hpp`

- 状态机管理: `IDLE → DETECTING → TRACKING ⇄ TEMP_LOST`
- 多靶匹配与叶片 ID 跟踪
- 基于马氏距离的观测门控

### 3. AngleFitter (异步拟合器)

`include/energy_meter_solver/angle_fitter.hpp`

支持两种运动模型：

**大能量机关 (正弦波模型)**
```
角度: θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
速度: v(t) = a·sin(ωt + t₀) + b

约束条件:
  a ∈ [0.780, 1.045] rad/s  (振幅)
  ω ∈ [1.884, 2.000] rad/s  (角频率)
  b = 2.090 - a             (速度偏移)
```

**小能量机关 (线性模型)**
```
角度: θ(t) = ω·t + b
恒定角速度: π/3 rad/s
```

关键特性：
- 异步拟合线程，不阻塞主循环
- 自适应 RANSAC 离群点剔除
- 两阶段网格搜索（粗→细）
- Levenberg-Marquardt 参数优化
- 自动大/小符类型检测（基于速度变化范围）

### 4. EnergyMeterSolverNode

`src/energy_meter_solver_node.cpp`

- ROS2 组件节点
- 订阅 `rune_detector/marker`，发布预测结果
- 自动模型类型判定（前25帧）
- 误判切换保护机制（冷却30帧）

## 依赖

- **ROS2** Humble+
- **Eigen3** 线性代数
- **tf2** 坐标变换
- **rm_interfaces** 自定义消息

## 安装

```bash
cd ~/ros2_ws/src
# energy_meter_solver 已存在
cd ..
colcon build --packages-select energy_meter_solver
source install/setup.bash
```

## 使用

### 启动节点

```bash
ros2 launch energy_meter_solver energy_meter_solver.launch.py
```

### 自定义参数

```bash
ros2 launch energy_meter_solver energy_meter_solver.launch.py \
  config_file:=/path/to/custom/params.yaml
```

### 直接运行

```bash
ros2 run energy_meter_solver energy_meter_solver_node \
  --ros-args \
  -p rune_type:=big \
  -p predict_time:=0.3 \
  -p auto_detect_type:=true
```

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `rune_detector/marker` | `visualization_msgs/MarkerArray` | R中心(SPHERE) + 目标靶(CUBE) |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `energy_meter_solver/state` | `rm_interfaces/EnergyMeterState` | 预测结果与模型参数 |
| `energy_meter_solver/marker` | `visualization_msgs/MarkerArray` | RViz 可视化 |

## 参数

详见 `config/params.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rune_type` | `"big"` | 默认类型（会被自动检测覆盖） |
| `predict_time` | `0.3` | 预测时间偏移 (s) |
| `gimbal_frame` | `"gimbal"` | 云台坐标系 |
| `world_frame` | `"odom"` | 世界坐标系 |
| `smooth_alpha` | `0.8` | 参数平滑系数 |
| `min_data_points` | `15` | 最小拟合点数 |
| `control_delay` | `0.05` | 控制延迟补偿 (s) |

## 预测算法

### 大符预测

```cpp
// 从当前速度反推相位
double sin_phi = (v_now - b) / a;
double phi_now = asin(clamp(sin_phi, -1, 1));

// 积分求角度增量
double phi_future = phi_now + omega * predict_time;
double delta_theta = -a/omega * cos(phi_future)
                   + a/omega * cos(phi_now)
                   + b * predict_time;
```

### 小符预测

```cpp
double delta_theta = omega * predict_time;
// 补偿一个靶位 (2π/5)
delta_theta += 2.0 * M_PI / 5.0;
```

## RViz 可视化

添加 MarkerArray 显示：
- **蓝绿色方块**: 其他 4 个靶位
- **黄色方块**: 预测位置 (`PREDICT`)

## 故障排查

### 无预测输出

1. 检查输入话题：
   ```bash
   ros2 topic hz rune_detector/marker
   ```

2. 检查模型状态：
   ```bash
   ros2 topic echo energy_meter_solver/state --field model_valid
   ```

### 预测偏差大

- 调整 `predict_time` 补偿系统延迟
- 增大 `min_data_points` 提高拟合精度
- 检查 `control_delay` 是否匹配实际云台延迟

## 性能指标

- 设计频率: ~30 Hz
- 拟合收敛: 约 20-30 帧
- 内存占用: ~5 MB 基础 + ~100 KB 历史缓存

## License

Apache License 2.0

## Authors

Vision Team - RoboMaster 2025
