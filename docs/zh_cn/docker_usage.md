# Build Docker Image

于 Repo 根目录下，在终端中使用：

```bash
docker build . -t atvision
```

构建开发容器和部署容器。

需要注意的是，部分国家和地区会阻断对 `github` 的连接，此时构建需要使用代理：

```bash
docker build . -t atvision \
--network host \
--build-arg HTTP_PROXY=http://127.0.0.1:7890 \
--build-arg HTTPS_PROXY=http://127.0.0.1:7890
```

如果上述方法行不通，
```bash
sudo vim /etc/docker/daemon.json
```

然后写入以下代理配置

```json
{
        "registry-mirrors": [
                "https://docker.m.daocloud.io",
                "https://docker.1panel.live",
                "https://hub.rat.dev"
        ]
}
```

如果该配置文件出错，docker 服务会无法再次启动。
最后，重启docker

```bash
sudo service docker restart
```

请自行把 `http://127.0.0.1:7890` 改为合适的代理地址。

如果不使用本机回环地址 (127.0.0.1) 作为代理地址，构建参数中 `--network host` 可以省去。