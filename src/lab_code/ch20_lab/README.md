# 第20章 实验代码：视觉大模型与 ROS2 应用

本章保留将视觉模型能力封装为 ROS2 服务的设计，以及离线 mock 与真实模型提供商的切换方式。离线 mock 不需要 API 密钥；真实模型通过参数选择提供商，并从环境变量读取密钥。

实现包：`src/lab_code/ch20_lab/vision_llm_demo/`（详见其 README）。
