# 实际运行证据

本页只登记由课程程序在当前环境中实际运行生成的输出。终端证据来自限时 `asciinema` 会话，再由 `scripts/render_asciinema_png.py` 渲染为 PNG；Gazebo 场景证据来自 Gazebo 的 Screenshot 插件。

采集环境为 WSL2 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim Harmonic。采集脚本为每个步骤设置硬超时：单步默认 8 秒、后台进程默认 40 秒、GUI 默认 45 秒。

## 已完成

| 章节/场景 | 实际证据 |
|:---|:---|
| ch01 生命周期节点 | [终端截图](images/runtime/ch01_lifecycle.png) · [原始录制](images/runtime/ch01_lifecycle.cast) |
| ch02 Python 节点 | [终端截图](images/runtime/ch02_nodes.png) · [原始录制](images/runtime/ch02_nodes.cast) |
| ch03 话题通信 | [终端截图](images/runtime/ch03_topics.png) · [原始录制](images/runtime/ch03_topics.cast) |
| ch04 服务通信 | [终端截图](images/runtime/ch04_service.png) · [原始录制](images/runtime/ch04_service.cast) |
| ch05 动作通信 | [终端截图](images/runtime/ch05_action.png) · [原始录制](images/runtime/ch05_action.cast) |
| ch06 参数系统 | [终端截图](images/runtime/ch06_parameters.png) · [原始录制](images/runtime/ch06_parameters.cast) |
| ch07 TF2 | [终端截图](images/runtime/ch07_tf.png) · [原始录制](images/runtime/ch07_tf.cast) |
| ch09 Gazebo headless | [终端截图](images/runtime/ch09_gazebo_headless.png) · [原始录制](images/runtime/ch09_gazebo_headless.cast) |
| Campus PUCRS Gazebo GUI | [真实场景截图](images/runtime/campus_pucrs_gazebo_gui.png) |
| Campus PUCRS headless | [终端截图](images/runtime/campus_pucrs_headless.png) · [原始录制](images/runtime/campus_pucrs_headless.cast) |
| ch26 控制器单元测试 | [终端截图](images/runtime/ch26_control.png) · [原始录制](images/runtime/ch26_control.cast) |

## 待采集

- ch08、ch10-ch25、ch27-ch31 仍需逐个运行并生成证据。
- CARLA 章节需要可用的 CARLA 服务器和 ROS bridge；当前环境未提供，不能伪造截图。
- RViz 已能启动并加载 OpenGL，但 WSLg 的窗口抓取返回全黑帧；黑帧不作为实验截图，需改用可见的桌面捕获链路后再登记。
