# vision_llm_demo — 第 20 章视觉大模型实验

- 包类型：`ament_python`
- ROS 2 Jazzy + std_srvs/Trigger 服务
- 提供商：`mock`（离线）/ `openai`（任意 OpenAI 兼容接口）

## 简介

本章将视觉大模型的看图描述能力封装为 ROS2 服务：`vision_llm_server`
对外提供 `/vision_llm`（`std_srvs/srv/Trigger`），每次调用返回一条场景
描述；`vision_llm_client` 演示多次调用。

| 程序 | 内容 |
|------|------|
| `vision_llm_server` | 视觉描述服务（参数 `provider` / `prompt` / `image_path`） |
| `vision_llm_client` | 调用服务并打印描述（参数 `count` / `interval_sec`） |

## 构建

```bash
cd <course_ws 工作区>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vision_llm_demo
source install/setup.bash
```

## 运行

```bash
# 离线演示（默认 mock，无需任何密钥）
ros2 run vision_llm_demo vision_llm_server
ros2 run vision_llm_demo vision_llm_client --ros-args -p count:=2

# 真实提供商：凭据只经环境变量注入，勿写入任何文件或文档
export VISION_API_KEY=<your-key>
export VISION_BASE_URL=<openai-compatible-base-url>
export VISION_MODEL=<model-name>
ros2 run vision_llm_demo vision_llm_server --ros-args -p provider:=openai
```

## 测试

```bash
colcon test --packages-select vision_llm_demo
colcon test-result --all
```

## 说明

- mock 描述以 `MOCK-VISION:` 为前缀并轮换输出，便于离线验证服务链路。
- 真实提供商需网络可达；凭据仅从 `VISION_API_KEY` 等环境变量读取，
  日志与文档中不出现明文密钥。
- 可选参数 `image_path` 指向本地图片时，`openai` 提供商会以 base64
  data URL 形式随请求上传。
