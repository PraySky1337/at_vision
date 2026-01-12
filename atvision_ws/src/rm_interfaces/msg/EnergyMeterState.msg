# Energy Meter Solver State Message
# 包含所有关键的求解状态信息，易于理解和调试

std_msgs/Header header

# 当前观察到的角度信息
float64 absolute_angle      # 原始角度（-π to π），单位：弧度
float64 continuous_angle    # 连续角度（持续增长，超过360°不重置）：0° → 90° → 180° → 270° → 360° → 450° ...，单位：弧度
int32 blade_offset          # 扇叶偏移计数（补偿叶片切换导致的角度跳变）

# 预测的角度和位置
float64 predicted_angle     # 预测的未来角度，单位：弧度
geometry_msgs/Vector3 predicted_position  # 预测的目标位置（世界坐标系），单位：米

# 几何信息（供MPC使用）
geometry_msgs/Vector3 r_center  # R标中心位置（世界坐标系），单位：米
float64 radius                  # 能量机关半径（R中心到靶心距离），单位：米

# 模型状态
bool model_valid            # 模型是否有效
bool is_big_rune            # 是否为大符
float64 fit_error           # 拟合误差
int32 data_points_used      # 用于拟合的数据点数

# 模型参数（大符：angle(t) = -a/ω·cos(ωt + t₀) + b·t + c，其中 b = 2.090 - a）
float64 sin_amplitude       # a: 振幅 [0.780, 1.045]
float64 sin_omega           # ω: 角频率（rad/s）[1.884, 2.000]
float64 sin_phase           # t₀: 相位偏移（rad）
float64 sin_offset          # b: 速度偏移 = 2.090 - a
float64 sin_const_term      # c: 常数项（rad）

# 模型参数（小符：angle(t) = ω·t + b）
float64 lin_omega           # ω: 角速度（rad/s）
float64 lin_offset          # b: 初始偏移（rad）

# 【新增】卡尔曼滤波后的速度和加速度
float64 filtered_velocity        # 卡尔曼滤波后的角速度（rad/s）

float64 predicted_time
