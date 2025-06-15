
---

[TOC]

# 前言

你有没有遇到过这种情况？

你兴冲冲地在Windows上用Docker搭了个**语音识别项目**，准备让容器高歌一曲，或者至少"Hey Docker"一下。结果——静音。

Docker Desktop一脸无辜："亲，默认配置里可没有'让你的容器唱歌'这个选项哦~"

于是，你的容器像个哑巴，明明代码写得飞起，却死活发不出声音。这感觉就像教鹦鹉说'Hello World'，但它死活不张嘴！

Docker：我聋了，也哑了
默认情况下，Docker容器在Windows上就像戴了降噪耳机——它听不见你，你也听不见它。

你想让它处理音频？它只会回你一个：

Error: No audio devices found. 
(内心OS：怪我咯？你也没给我麦克风啊！)

别急，今天我们就来教Docker如何"开口说话"，让它能抢到Windows的麦克风和扬声器，让你的语音项目不再是个"哑剧演员"！

> 参考
> - [Docker Tips - Play Audio in a Container (2022)](https://www.youtube.com/watch?v=SF_WMBpQ0Qs)

# 操作配置

1.下载pulseaudio服务

[https://www.freedesktop.org/wiki/Software/PulseAudio/Ports/Windows/Support/](https://www.freedesktop.org/wiki/Software/PulseAudio/Ports/Windows/Support/)

![alt text](images/image.png)

并解压文件夹到D盘

![alt text](images/image-1.png)

2.配置pulseaudio服务

![alt text](images/image-2.png)

```shell
load-module module-native-protocol-tcp listen=0.0.0.0 auth-anonymous=1
```

3.启动pulseaudio服务

> 注意：宿主机重启后，需要重新启动pulseaudio服务。


```shell
# 加-D 表示后台运行
.\bin\pulseaudio.exe --use-pid-file=false -D
```

![alt text](images/image-4.png)

允许一下

![alt text](images/image-3.png)

4.配置docker容器

```shell
docker run -it -p 6080:80 -p 10000:10000 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 -e PULSE_SERVER=host.docker.internal --name=DockerSpeaker docker.1ms.run/ubuntu:24.04  /bin/bash
```

5.测试

```shell
apt-get update && apt-get install -y alsa-utils pulseaudio
aplay -L  # 列出音频设备
speaker-test -t wav  # 测试播放
```

![alt text](images/image-5.png)

测试成功！