# 免密登录

```sh
ssh-keygen -t rsa -b 2048 -N "" -f ~/.ssh/id_rsa -q
ssh-copy-id -i ~/.ssh/id_rsa.pub developer@xxx.xxx.xxx.xx
```
