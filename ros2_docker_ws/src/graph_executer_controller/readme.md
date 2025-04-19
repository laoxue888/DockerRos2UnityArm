
---

[TOC]

# 前言

配置运行graph_executer_controller的开发环境


# 操作

❇️配置开发环境

```shell
apt install python3-pip -y

# 使用清华源下载
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages
```

❇️运行

```shell
cd src
source install/setup.bash
cd graph_executer_controller
python3 main.py
```
