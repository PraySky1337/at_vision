FROM ros:humble AS atvision

LABEL maintainer="3159890292@qq.com" \
      version="1.0-allinone"  \
      description="ATVISION dev + runtime image with ROS2 + deps"

SHELL ["/bin/bash", "-c"]
ENV TZ=Asia/Shanghai DEBIAN_FRONTEND=noninteractive
ARG CLANG_VERSION=20

COPY atvision_ws/src /tmp/atvision_ws/src
RUN apt-get update && apt-get install -y \
    # 基础工具
    build-essential gdb cmake git curl wget htop vim \
    usbutils net-tools iputils-ping openssh-server \
    # 项目依赖
    libusb-1.0-0-dev \
    libfmt-dev \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    libspdlog-dev \
    libsuitesparse-dev \
    # Foxglove-Bridge
    ros-humble-foxglove-bridge \
    # g2o 依赖
    qtbase5-dev \
    qtdeclarative5-dev \
    qttools5-dev-tools \
    libqglviewer-dev-qt5 \
    # 其他
    tini \
    tmux \
    sudo \
    python3-colorama \
    python3-dpkt \
 && rosdep install --from-paths /tmp/atvision_ws/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/* /tmp/atvision_ws/src

# g2o
RUN git clone --depth=1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cmake -S /tmp/g2o -B /tmp/g2o/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/g2o/build --parallel $(nproc) && \
    cmake --install /tmp/g2o/build && \
    rm -rf /tmp/g2o

# openvino
RUN wget -qO - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor -o /usr/share/keyrings/intel-openvino.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-openvino.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino-2024.list && \
    apt-get update && apt-get install -y openvino-2024.6.0 && rm -rf /var/lib/apt/lists/*

# llvm
RUN wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | \
    gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] \
      http://apt.llvm.org/$(lsb_release -cs)/ llvm-toolchain-$(lsb_release -cs)-$CLANG_VERSION main" \
      > /etc/apt/sources.list.d/llvm-apt.list && \
    apt-get update && apt-get install -y \
      clang-$CLANG_VERSION \
      clangd-$CLANG_VERSION \
      clang-format-$CLANG_VERSION \
      clang-tidy-$CLANG_VERSION \
      gcc-12 g++-12 ninja-build && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-$CLANG_VERSION 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$CLANG_VERSION 50 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-$CLANG_VERSION 50 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-$CLANG_VERSION 50 && \
    rm -rf /var/lib/apt/lists/*

# ---------- 用户与权限配置 ----------
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 创建用户并保持 UID/GID 对齐宿主机（防止 Volume 挂载卷权限异常)
RUN if getent passwd $USER_UID >/dev/null; then \
      # 已存在 UID=1000 用户（例如 ubuntu），则直接复用并改名为 developer
      OLDUSER=$(getent passwd $USER_UID | cut -d: -f1) && \
      usermod -l $USERNAME -d /home/$USERNAME -m $OLDUSER && \
      groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) || true; \
    else \
      # 否则新建用户
      groupadd -g $USER_GID $USERNAME && \
      useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME; \
    fi && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add $USERNAME dialout && \
    chsh -s /bin/bash $USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

COPY --chown=root:root --chmod=755 script/atvision/build-atv /usr/local/bin/build-atv
COPY --chown=root:root --chmod=755 script/atvision/entrypoint /usr/local/bin/entrypoint
COPY --chown=root:root --chmod=755 script/atvision/setupenv /usr/local/bin/setupenv

ENV IN_DOCKER=true
ENV UDEV=1


ENTRYPOINT ["tini", "--"]
CMD ["entrypoint"]
