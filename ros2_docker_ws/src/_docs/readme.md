
---

[TOC]

# 前言

unity和ros2的通信测试。

# 测试

```shell
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.launch.py
```

```shell
source install/setup.bash
ros2 run unity_control_example follow_joint_trajectory_monitor
```

```shell
source install/setup.bash
ros2 launch niryo_one_moveit_config demo.launch.py
```
