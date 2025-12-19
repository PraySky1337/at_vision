# rm_calibration

基于 ROS2 的相机标定工具，按本项目（圆点阵列 + OpenCV calibrateCamera/solvePnP/calibrateHandEye）的逻辑实现：

- 订阅 `image_topic`（默认 `/image_raw`）
- 订阅 `camera_info_topic`（默认 `/camera_info`，外参标定会使用其中的内参/畸变）
- 通过 TF2 查询 `base_frame`（默认 `odom`）到 `gimbal_frame`（默认 `gimbal_link`）的变换（实际使用 `lookupTransform(base_frame, gimbal_frame, stamp)`）
- 提供两个服务：`capture`、`calibrate`

标定板检测：默认同时兼容 **对称圆点阵列**（`findCirclesGrid`）和 **棋盘格角点**（`findChessboardCorners`），任一成功即可参与标定；`board_width/board_height` 表示“圆心/内角点”的列数/行数。

## 运行

```bash
ros2 launch rm_calibration calibration.launch.py
```

默认参数见 `rm_calibration/config/calibration_params.yaml`。

## 服务

### 1) Capture

采集一帧图像，并查询该图像时间戳对应的 TF（`base_frame <- gimbal_frame`），保存到本地目录 `data_dir`。

```bash
ros2 service call /calibration_node/capture std_srvs/srv/Trigger {}
```

输出文件：
- `data_dir/000001.<image_ext>`：图像
- `data_dir/000001.txt`：对应 TF（平移 + 四元数，四元数顺序为 `wxyz`）

### 2) Calibrate

对 `data_dir` 内的文件进行标定；由参数 `mode` 决定标定类型：
- `mode=intrinsic`：内参标定（仅使用图像）
- `mode=extrinsic`：外参标定（使用图像 + `*.txt` + `/camera_info` 的内参）

```bash
ros2 param set /calibration_node mode intrinsic
ros2 service call /calibration_node/calibrate std_srvs/srv/Trigger {}
```

```bash
ros2 param set /calibration_node mode extrinsic
ros2 service call /calibration_node/calibrate std_srvs/srv/Trigger {}
```

输出：
- 内参：默认写入 `data_dir/intrinsic.yaml`（可用 `intrinsic_output_path` 覆盖）
- 外参：默认写入 `data_dir/extrinsic.yaml`（可用 `extrinsic_output_path` 覆盖），包含 `R_camera2gimbal` / `t_camera2gimbal`

## 主要参数

- `base_frame`：如 `odom`
- `gimbal_frame`：如 `gimbal_link`（兼容旧参数名 `imu_frame`）
- `image_topic`：如 `/image_raw`
- `camera_info_topic`：如 `/camera_info`
- `board_width`/`board_height`：圆点阵列列数/行数（默认 `10x7`）
- `square_size`：相邻圆心距离（单位 m）
- `use_gripper_translation`：外参标定是否使用 TF 的平移量
