std_msgs/Header header

# 原始弹道解算结果（无MPC规划）
float32 raw_yaw           # 原始yaw目标角（rad）
float32 raw_pitch         # 原始pitch目标角（rad）

# MPC规划后的参考轨迹（第一步）
float32 ref_yaw           # 规划后yaw位置（rad）
float32 ref_yaw_vel       # 规划后yaw速度（rad/s）
float32 ref_yaw_acc       # 规划后yaw加速度（rad/s²）
float32 ref_pitch         # 规划后pitch位置（rad）
float32 ref_pitch_vel     # 规划后pitch速度（rad/s）
float32 ref_pitch_acc     # 规划后pitch加速度（rad/s²）

# 当前云台状态
float32 cur_yaw           # 当前yaw角（rad）
float32 cur_yaw_vel       # 当前yaw角速度（rad/s）
float32 cur_pitch         # 当前pitch角（rad）
float32 cur_pitch_vel     # 当前pitch角速度（rad/s）

# 状态信息
bool mpc_valid            # MPC求解是否有效
bool fire_advice          # 开火建议
float32 distance          # 目标距离（m）
