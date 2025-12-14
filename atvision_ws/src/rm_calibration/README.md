# Camera-IMU Calibration Package

ROS2功能包，用于相机与IMU的联合标定。

## 功能特性

- **双模式支持**：内参标定和外参标定
- **基于OpenCV**：使用成熟的标定算法
- **ROS2集成**：完全集成TF2和标准ROS2接口
- **实时反馈**：提供可视化反馈和进度显示

## 依赖项

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge \
                 ros-${ROS_DISTRO}-image-transport \
                 ros-${ROS_DISTRO}-tf2 \
                 ros-${ROS_DISTRO}-tf2-ros \
                 libopencv-dev
```

## 编译

```bash
cd ~/ros2_ws/src
git clone <repository_url> camera_imu_calibration
cd ~/ros2_ws
colcon build --packages-select camera_imu_calibration
source install/setup.bash
```

## 使用方法

### 步骤1: 准备标定板

准备一个棋盘格标定板（推荐：10x7格子，每格2.5cm）

### 步骤2: 内参标定

1. **启动内参标定模式**：

```bash
ros2 launch camera_imu_calibration calibration.launch.py mode:=intrinsic
```

2. **采集图像**：
   - 节点启动后会持续检测棋盘格并实时显示角点，但不会自动保存
   - 调用标定服务后进入采集模式（默认30s），自动挑选质量最高的样本
   - 在采集窗口内移动标定板，覆盖不同距离、角度和位置
   - 默认收集最优25帧（可在配置文件中调整）

3. **执行标定**：

```bash
ros2 service call /calibration_node/calibrate std_srvs/srv/Trigger
```

   服务会启动采集，倒计时结束后自动输出标定结果。

4. **保存结果**：
   - 标定结果会输出到终端
   - 将`camera_matrix`和`dist_coeffs`复制到配置文件中

示例输出：
```
=== Camera Intrinsic Calibration Results ===
Camera Matrix K:
[800.123, 0.000, 320.456]
[0.000, 799.876, 240.321]
[0.000, 0.000, 1.000]

Distortion Coefficients D:
[-0.123, 0.045, 0.001, -0.002, 0.000]
```

### 步骤3: 外参标定

1. **更新配置文件**：
   - 将内参标定的结果填入`config/calibration_params.yaml`
   - 设置正确的TF frame名称

```yaml
camera_matrix: [800.123, 0.0, 320.456, 0.0, 799.876, 240.321, 0.0, 0.0, 1.0]
dist_coeffs: [-0.123, 0.045, 0.001, -0.002, 0.000]
imu_frame: "gimbal_link"
base_frame: "odom"
```

2. **启动外参标定模式**：

```bash
ros2 launch camera_imu_calibration calibration.launch.py mode:=extrinsic
```

3. **采集样本**：
   - 确保IMU在发布TF变换（`odom` → `gimbal_link`）
   - 调用`~/calibrate`服务后进入采样窗口，自动保留最优样本
   - 在采集过程中移动标定板并改变IMU姿态，保证姿态多样性
   - 默认目标25个高质量样本（可参数化）

4. **执行标定**：

```bash
ros2 service call /calibration_node/calibrate std_srvs/srv/Trigger
```

   服务会启动采集并在窗口结束后自动输出标定结果。

5. **查看结果**：

```
=== Camera-IMU Extrinsic Calibration Results ===
Rotation Matrix R (Camera to IMU):
[0.999, -0.012, 0.034]
[0.013, 0.999, -0.045]
[-0.033, 0.045, 0.998]

Translation Vector t (Camera to IMU):
[0.050, 0.020, -0.030]
```

## 话题和服务

### 订阅的话题

- `/image_raw` (sensor_msgs/Image): 相机图像

### TF变换（仅外参模式）

- `odom` → `gimbal_link`: IMU姿态

### 服务

- `~/calibrate` (std_srvs/Trigger): 执行标定计算
- `~/reset` (std_srvs/Trigger): 清除已采集的数据

## 参数配置

在`config/calibration_params.yaml`中配置：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| mode | 标定模式：intrinsic/extrinsic | intrinsic |
| board_width | 棋盘格内部角点列数 | 9 |
| board_height | 棋盘格内部角点行数 | 6 |
| square_size | 每个方格的边长（米） | 0.025 |
| calibration_duration | 采集时长（秒） | 30.0 |
| target_samples | 目标样本数量（Top-N保存） | 25 |
| quality_threshold | 样本质量阈值（0-1） | 0.6 |
| display_fps | 叠加状态刷新率 | 10 |
| required_frames | 兼容旧配置，不再直接使用 | 20 |
| imu_frame | IMU的TF frame | gimbal_link |
| base_frame | 基础TF frame | odom |
| camera_matrix | 相机内参矩阵（3x3展开） | - |
| dist_coeffs | 畸变系数（5个） | - |
| weight_corner_quality | 内参：角点质量权重 | 0.3 |
| weight_board_size | 内参：标定板大小权重 | 0.2 |
| weight_board_angle | 内参：角度/分布权重 | 0.3 |
| weight_image_sharpness | 内参：清晰度权重 | 0.2 |
| weight_reprojection_error | 外参：重投影误差权重 | 0.4 |
| weight_imu_diversity | 外参：IMU姿态多样性权重 | 0.3 |
| weight_board_pose | 外参：标定板姿态权重 | 0.3 |

## 运行时切换模式

```bash
# 切换到外参标定模式
ros2 param set /calibration_node mode extrinsic

# 重置数据
ros2 service call /calibration_node/reset std_srvs/srv/Trigger
```

## 标定建议

### 内参标定技巧

1. 采集覆盖整个图像区域的图像
2. 包含不同距离的图像（近、中、远）
3. 包含不同角度的图像（正面、倾斜）
4. 确保标定板清晰可见，无模糊

### 外参标定技巧

1. 确保IMU数据稳定可靠
2. 移动标定板时保持IMU相对稳定
3. 采集多种IMU姿态下的数据
4. 标定板应在相机视野内清晰可见

## 故障排除

### 问题：无法检测到标定板

**解决方案**：
- 确保光照充足
- 调整标定板大小参数
- 检查图像是否清晰

### 问题：标定精度不高

**解决方案**：
- 增加采集的帧数/样本数
- 确保数据多样性
- 检查标定板尺寸测量是否准确

### 问题：无法获取TF变换

**解决方案**：
- 检查TF frame名称是否正确
- 确认IMU节点正在发布TF
- 使用`ros2 run tf2_tools view_frames`查看TF树

## 输出格式

标定结果可以直接用于：
- ROS camera_info消息
- OpenCV相机模型
- SLAM/视觉惯导系统

## 许可证

MIT License

## 作者

PraySky && Claude && Codex

## 贡献

欢迎提交Issue和Pull Request！
