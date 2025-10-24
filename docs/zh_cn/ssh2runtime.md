# 开发容器向部署容器的SSH

*我们的一切操作都基于开发容器向部署容器的 SSH 连接*

开发容器向部署容器进行 SSH 操作 并通过`rsync`软件单向增量同步整个项目到目标主机，支持直接自主编译，也可以在对方会话进行编译
在第一次ssh时，先用`ssh-copy-id`注册免密ssh，随后才可以正常使用`rsync`

```sh
ssh-copy-id -i ~/.ssh/id_rsa.pub -p 2025 developer@localhost 
```

同步命令
```

```
