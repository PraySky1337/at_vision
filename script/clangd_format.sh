# ① 拉取并启用官方源
wget -qO- https://apt.llvm.org/llvm.sh | sudo bash

# ② 安装 clang-format‑19 和 clangd‑19
sudo apt update
sudo apt install -y clang-format-19 clangd-19

# ③ 把 clangd‑19 设为系统缺省
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 100

# （如系统仍有旧版 clangd，可交互切换）
sudo update-alternatives --config clangd
