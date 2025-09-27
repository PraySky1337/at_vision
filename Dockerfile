# ---------- 基础阶段 ----------
FROM ros:humble

LABEL maintainer="3159890292@qq.com" \
      version="1.0-allinone" \
      description="ATVISION dev + runtime image with ROS2 + deps"

SHELL ["/bin/bash", "-c"]

ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# apt 优化：不装推荐/建议包
RUN echo 'APT::Install-Recommends "false";\nAPT::Install-Suggests "false";' \
    > /etc/apt/apt.conf.d/99no-recommends

# 基础依赖
RUN apt-get update && apt-get install -y \
    build-essential gdb cmake git vim curl wget htop usbutils net-tools iputils-ping openssh-server \
    libusb-1.0-0-dev \
    libfmt-dev \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# ---------- 第三方库 ----------
WORKDIR /home/third-party

# 拷贝源码，仅用于依赖解析
COPY atvision_ws/src /tmp/atvision_ws/src
RUN apt-get update && \
    rosdep install --from-paths /tmp/atvision_ws/src --ignore-src -r -y && \
    rm -rf /tmp/atvision_ws/src && \
    rm -rf /var/lib/apt/lists/*

# g2o
COPY third-party/g2o /tmp/g2o
RUN apt-get update && apt-get install -y \
      libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 && \
    cd /tmp/g2o && \
    cmake -B build && \
    cmake --build build -j$(nproc) && \
    sudo cmake --install build && \
    rm -rf /tmp/g2o && rm -rf /var/lib/apt/lists/*

# OpenVINO runtime
RUN wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
    | gpg --dearmor -o /usr/share/keyrings/intel-openvino.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-openvino.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
    > /etc/apt/sources.list.d/intel-openvino-2024.list && \
    apt-get update && apt-get install -y openvino-2024.6.0 && \
    rm -rf /var/lib/apt/lists/*

# ---------- 开发工具链 ----------
RUN apt-get update && apt-get install -y \
    libc6-dev gcc-12 g++-12 cmake make ninja-build \
    openssh-client lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt && \
    wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key \
    | gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] http://apt.llvm.org/jammy/ llvm-toolchain-jammy main" \
    > /etc/apt/sources.list.d/llvm-apt.list && \
    apt-get update && \
    version=$(apt-cache search clangd- | grep clangd- | awk '{print $1}' | sort -V | tail -1 | cut -d- -f2) && \
    apt-get install -y clangd-$version && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$version 50 && \
    rm -rf /var/lib/apt/lists/*

# ---------- 用户 ----------
RUN useradd -m developer --shell /bin/bash && \
    echo "developer:developer" | chpasswd && adduser developer sudo && \
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add developer dialout
USER developer
WORKDIR /home/ws

ENV USER=developer \
    WORKDIR=/home/at_vision
# ---------- 默认入口 ----------
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
