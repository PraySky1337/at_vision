#!/bin/bash
set -e

echo "[INFO] 创建 udev 规则文件 /etc/udev/rules.d/99-usb-devices.rules"

sudo tee /etc/udev/rules.d/99-usb-devices.rules >/dev/null <<EOF
# Hikrobot industrial camera (idVendor=2bdf)
SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"

# STM32 Virtual ComPort (idVendor=0483)
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"

# 可在这里继续加其它 USB 设备，比如 IMU、相机等
EOF

echo "[INFO] 重新加载 udev 规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[INFO] 设置完成，当前已加载规则："
ls -l /etc/udev/rules.d/99-usb-devices.rules
