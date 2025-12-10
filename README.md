# ATVISION
RoboMaster Vision System Based On ROS2
![Actor Thinker](docs/pic/actor_thinker.png)

## 简介
主要框架基于 rm_vision ，fyt_vision , 适配队内车车的视觉系统

### Development
持续开发中

#### Requirements

- x86-64
- Linux

#### Docker Build

由于Docker对部分国家地区有访问限制，需要手动配置代理，
且Ubuntu22.04LTS默认安装的Docker版本低于所需的构建版本(Mount功能缺失)，
所以请按照[Docker安装并配置代理](docs/zh_cn/docker_install.md)安装最新版的Docker，并配置代理。
Docker安装完成后，构建即可。

```sh
docker build . --target atvision-runtime -t atvision-runtime:latest
```

```sh
docker build . --target atvision-dev -t atvision-dev:latest;
```

如果出现 docker pull 失败的情况，建议[手动配置镜像源或使用代理](docs/zh_cn/docker_usage.md)

随后 导出runtime镜像
并通过scp或其他方式将 `atvisoin-runtime.tar` 传输到nuc上

```sh
docker save atvision-runtime:latest > atvision-runtime.tar
scp atvision-runtime.tar nuc@nuc.local:/home/nuc/Desktop
```

在nuc上执行以下命令

```sh
docker load -i atvision-runtime.tar
```

在目标机器上自动启动
```sh
docker run -d --restart=unless-stopped --privileged --network=host -v /dev:/dev atvision-runtime:latest
```

#### Container dev

安装 DevContainer

```sh
code --install-extension ms-vscode-remote.remote-containers
```

或自己手动安装
安装完成后，在项目(atvision)的根目录下，摁ctrl+shift+p打开命令页面
选择 "在容器中重新打开"。随后等待进入容器即可。

进入容器后，可选./vscode中的settings.default.json作为推荐设置进行开发


#### Runtime - intel激活OV

```sh
# activate ov gpu 目前只支持 x86_64 
mkdir -p ~/neo && \
cd ~/neo && \
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-core_1.0.13463.18_amd64.deb  &&\
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-opencl_1.0.13463.18_amd64.deb &&\
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu-dbgsym_1.3.25812.14_amd64.ddeb &&\ 
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu_1.3.25812.14_amd64.deb  &&\
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd-dbgsym_23.09.25812.14_amd64.ddeb  &&\
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd_23.09.25812.14_amd64.deb  &&\
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/libigdgmm12_22.3.0_amd64.deb  &&\
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/ww09.sum &&\
sha256sum -c ww09.sum &&\
sudo dpkg -i *.deb &&\
rm -rf ~/neo
cd /home/developer/ws
```

#### ROS2

关于[日志系统](https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html#id6)

```bash
# 控制台输出着色
export RCUTILS_COLORIZED_OUTPUT=1  # 1 for forcing it
# 控制台输出格式
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export RCUTILS_LOGGING_USE_STDOUT=0 # 在Jazzy及以上，默认为stderr流而不是stdout，这个可以不改
export RCUTILS_LOGGING_BUFFERED_STREAM=0 # 不进行缓冲，不建议修改
```


## 一些待完善的工作

- [ ] 将Solver与状态估计器解耦，在rm_gimbal包中添加二次规划功能

- [ ] ros bag 回放

- [ ] 全向感知

- [ ] 在线手眼标定 from rm_calibration

- [x] 更准确的弹道解算(考虑枪口系)

- [x] 调整 UKF

- [ ] 自动测定usb通讯回环延迟以确定时间戳偏移量

- [x] 模拟视频发布 pub_video 节点实现

- [ ] 补充开发/使用文档

- [x] 仿真 基于 Bevy(rust) 的 bevy_rm_simulator 非常感谢  Black Jack

- [ ] 重新构建 Docker 镜像

- [x] 将通信中间件替换为 Zehno
    另注: 已经放弃该方案。因其使用zehno_cpp服务进程，本质中心化。
    目前的工作是将关键瓶颈升级为内存通信

- [ ] openvino版本升级为 25 版

- [ ] 删除 rm_utils 和心跳 非常糟糕的软件设计。