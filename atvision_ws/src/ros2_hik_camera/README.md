# ros2_hik_camera

A ROS2 packge for Hikvision USB3.0 industrial camera

## Usage

```
ros2 launch hik_camera hik_camera.launch.py
```

## Params

- exposure_time
- gain
- force_8bit_pixel_format (bool, default: true) - force the camera pixel format to an 8-bit variant (e.g., BayerRG8/Mono8) to reduce bandwidth

## Multi-platform build

The build auto-detects the host architecture (amd64/arm64) and links the matching Hik SDK libs. For cross-builds or Docker Buildx, set `HIK_SDK_ARCH` to pick the target SDK variant:

```
colcon build --cmake-args -DHIK_SDK_ARCH=arm64
```
