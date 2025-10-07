# Script

## atvision folder

atvision folder中的脚本是在Dockerfile构建的时候安装到 /usr/local/bin 并被赋予权限的
后续的更新则需重建镜像。

- [build-atv](atvision/build-atv) 构建时请直接调用该工具，ARG详见脚本内部
- [entry-point](atvision/entrypoint) 进入容器时的默认行为（使用devcontainers时不会生效）。
- [setupenv](atvision/setupenv) 设置 ROS 和 项目环境


使用`build-atv`构建，才是默认方案，直接使用`colcon build`的方式也被支持，但建议仅用于调试。

`build-atv`会将构建中间产物build置于工作目录下（以确保compile_commands.json被正确使用）
install和log会默认置于容器的`/home/atvruntime/`目录中