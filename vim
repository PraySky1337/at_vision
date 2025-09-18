# /etc/systemd/system/vision.service
[Unit]
Description=Armor Detector ROS2 Node
After=network.target

[Service]
# 以 nuc 用户运行
User=nuc
# 设置环境变量
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1
# 工作目录
WorkingDirectory=/home/nuc/Desktop/vision
# 在 ExecStart 里先 source，再 launch
ExecStart=/bin/bash -lc '\
  source /opt/ros/humble/setup.bash && \
  source /home/nuc/Desktop/at_vision/install/setup.bash && \
  ros2 launch rm_auto_aim rm_auto_aim.launch.py'
# 失败后自动重启
Restart=on-failure
RestartSec=3
# 打开更多文件描述符
LimitNOFILE=65535

[Install]
WantedBy=multi-user.target

