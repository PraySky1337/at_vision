# Docker With Proxy

## Ubuntu 安装 Docker

在终端中运行

```bash
sudo apt-get update &&
sudo apt-get -y install ca-certificates curl &&
sudo install -m 0755 -d /etc/apt/keyrings &&
sudo curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc &&
sudo chmod a+r /etc/apt/keyrings/docker.asc &&
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://mirrors.aliyun.com/docker-ce/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&
sudo apt-get update &&
sudo apt-get -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

## 将用户添加至 Docker 组 (运行 docker 不再需要 sudo)

```sh
sudo usermod -aG docker $USER
newgrp docker
```

验证是否成功
```sh
groups $USER | grep "docker"
```

## 设置 docker pull 代理

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d &&
sudo tee /etc/systemd/system/docker.service.d/proxy.conf <<EOF
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:7890/"
Environment="HTTPS_PROXY=http://127.0.0.1:7890/"
EOF
sudo systemctl daemon-reload &&
sudo systemctl restart docker
```

默认代理地址为 127.0.0.1:7890
