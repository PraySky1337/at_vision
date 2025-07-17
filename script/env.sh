echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/50-stm32-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo 'export ROS_DOMAIN_ID=10' >> ~/.bashrc && source ~/.bashrc
