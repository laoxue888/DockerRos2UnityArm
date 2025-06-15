
---

# 前言

![alt text](images/image.png)

最近有小伙伴疯狂@鱼香ROS说一键安装程序抽风了。一开始我以为会很快修复，但是没想到的是，这个bug竟然持续了将近半个月。小鱼你快点修复bug吧，你这个一键安装可太好用了哇，为了不影响大家的学习，这里我简单介绍一个非常原始的ros2安装方法。虽然安装速度有点慢，但是总归是安装上了。

![alt text](images/wandiannao_xiao.gif)


# 环境配置

> 环境要求：
> - ubuntu: 24.04

这里以安装ros2 jazzy为例

❇️Ubuntu换源

```shell
apt update
apt install vim -y

# 备份
cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak

vim /etc/apt/sources.list.d/ubuntu.sources
```

用以下内容覆盖文件

```shell
# 阿里云
Types: deb
URIs: http://mirrors.aliyun.com/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

#####################或者清华源############################
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

#####################或者中科大源############################
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
Types: deb
URIs: http://mirrors.ustc.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

#####################或者网易163源############################
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
Types: deb
URIs: http://mirrors.163.com/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
```

❇️安装ros2

```shell
apt update

apt-get install sudo -y

sudo apt install software-properties-common -y

sudo add-apt-repository universe # 按后面Enter

sudo apt update && sudo apt install curl gnupg2 -y

sudo curl -sSL https://gitee.com/tyx6/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt install curl gnupg2 -y

# 这一步先打开科学上网，不然会报错
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 这一步先关闭科学上网，不然会报错
sudo apt update && sudo apt upgrade

sudo apt install ros-jazzy-desktop -y
```

❇️添加终端启动sh

```shell
echo "source /opt/ros/jazzy/setup.bash" > ~/.bashrc
```


❇️安装必要的第三方库

```shell
sudo apt install python3-colcon-common-extensions -y   
```

# 参考

- ROS2 软件仓库：https://mirror.tuna.tsinghua.edu.cn/help/ros2/
- ROS2 Installtion：https://docs.ros.org/en/humble/Installation.html
- Ubuntu 清华源：https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/
- Ubuntu 24.04 抢先体验换国内源 清华源 阿里源 中科大源 163源：https://blog.csdn.net/xiangxianghehe/article/details/136529419