FROM ros:humble AS atvision-base
LABEL maintainer="3159890292@qq.com" version="1.0-base" description="ATVISION base (ROS2 + OpenVINO + g2o + GCC)"
SHELL ["/bin/bash","-c"]
ENV TZ=Asia/Shanghai DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV IN_DOCKER=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget vim htop \
    usbutils net-tools iputils-ping \
    libusb-1.0-0-dev \
    libfmt-dev \
    libceres-dev \ 
    libeigen3-dev \
    libopencv-dev \
    libspdlog-dev \
    libsuitesparse-dev \
    qtbase5-dev qtdeclarative5-dev qttools5-dev-tools libqglviewer-dev-qt5 \
    tmux screen rsync

# rosdep
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
COPY atvision_ws/src /tmp/atvision_ws/src
RUN rosdep install --from-paths /tmp/atvision_ws/src --ignore-src -r -y \
    && rm -rf /tmp/atvision_ws/src /var/lib/apt/lists/*

# openvino
RUN wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /usr/share/keyrings/intel-openvino.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-openvino.gpg] https://apt.repos.intel.com/openvino/2023 ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && \
    apt-get install -y openvino-2023.3.0 && \
    rm -rf /var/lib/apt/lists/*

# g2o
RUN git clone --depth=1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cmake -S /tmp/g2o -B /tmp/g2o/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/g2o/build --parallel 6 && \
    cmake --install /tmp/g2o/build && rm -rf /tmp/g2o

# 统一用户（dev/runtime 继承一致）
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} && \
    echo "${USERNAME}:aaa" | chpasswd && \
    echo "root:aaa" | chpasswd \
    echo "${USERNAME} ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add ${USERNAME} dialout && \
    gpasswd --add ${USERNAME} plugdev

USER ${USERNAME}
WORKDIR /home/${USERNAME}

FROM atvision-base AS atvision-dev
LABEL version="1.0-dev" description="ATVISION dev (clang/llvm + rosdep + tools)"
SHELL ["/bin/bash","-c"]
USER root
ARG CLANG_VERSION=20

# 开发工具 & LLVM/Clang
RUN apt-get update && apt-get install -y --no-install-recommends \
    ninja-build python3-colorama python3-dpkt \
    sudo openssh-client\
 && rm -rf /var/lib/apt/lists/*
RUN wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] http://apt.llvm.org/$(lsb_release -cs)/ llvm-toolchain-$(lsb_release -cs)-${CLANG_VERSION} main" \
      > /etc/apt/sources.list.d/llvm-apt.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      clang-${CLANG_VERSION} \
      clangd-${CLANG_VERSION} \
      clang-format-${CLANG_VERSION} clang-tidy-${CLANG_VERSION} \
      gcc-12 g++-12 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 \
    && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${CLANG_VERSION} 50 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${CLANG_VERSION} 50 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${CLANG_VERSION} 50 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${CLANG_VERSION} 50 \
    && rm -rf /var/lib/apt/lists/*


USER developer
WORKDIR /home/developer

# runtime
FROM atvision-base AS atvision-runtime
LABEL version="1.0-runtime" description="ATVISION runtime (Foxglove + install; inherits compiler)"
SHELL ["/bin/bash","-c"]
USER root

# Network
RUN apt update && apt install -y openssh-server tini ros-$ROS_DISTRO-foxglove-bridge &&\
    echo 'Port 2025' >> /etc/ssh/sshd_config && \
    rm -rf /var/lib/apt/lists/*

# script
COPY --chmod=755 script/entrypoint  /usr/local/bin/entrypoint
COPY --chmod=755 script/atvision-service  /usr/local/bin/atvision-service

WORKDIR /root/
ENTRYPOINT [ "tini", "--" ]
CMD ["entrypoint"]
