# 设置容器开机自启动

该容器设计思想为 Dev 和 Runtime 不分离，降低复杂度的同时以加速迭代（但对于大型的项目这不是最佳实践）
Dev 以挂载的形式开发将内容长期保留在宿主机，方便迭代 Dockerfile
构建时 以 build.sh 脚本构建 将内容保存在容器的 "/home/atvruntime" 文件夹中。
Docker自启动时会尝试寻找 "/home/atvruntime/install/setup.zsh"，如果存在则启动
ros2 launch rm_vision

## 查看 Docker service 是否在线

```sh
$ systemctl list-unit-files | grep docker
```

如果无异常，应有如下输出
```sh
docker.service enable enable
docker.socket  enable enable
```

如果为disable，请设置为开机自启。

```sh
systemctl enable docker.service
```

## 设置容器开机自启

```sh
docker update --restart=always atvision
```
