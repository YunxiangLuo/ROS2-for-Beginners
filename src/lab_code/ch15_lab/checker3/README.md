# checker3 — 串口设备检测与 udev 映射工具

`checker3` 是一个 Linux 下的串口设备检测脚本，用于自动识别连接到开发板的 USB 串口设备并配置 udev 端口映射规则。

## 功能

1. **检测摄像头设备**：区分人脸识别摄像头（USB VID 0bda）和 Intel RealSense 摄像头
2. **检测激光雷达**：识别标签为 `CP2102` 的串口设备，映射为 `/dev/rplidar`
3. **检测机械臂**：识别标签为 `Serial Cable` 的串口设备，写入芯片序列号，映射为 `/dev/arm`
4. **检测 PCB1 板**：读取串口数据帧，校验协议头部 `aa5530`，解析编码器数据，映射为 `/dev/xbot`
5. **检测 PCB2 板**：读取串口数据帧，校验协议头部 `aa5538`，解析 9 轴 IMU 数据（加速度、角速度、磁场、欧拉角、四元数），映射为 `/dev/sensor`

## 使用方法

```bash
# 确保串口设备已连接
sudo python3 checker3
```

## 在 ROS2 中的使用

该工具生成的 udev 规则确保串口设备每次接入时都有固定的设备名，便于 ROS2 节点配置：

```bash
# 创建 udev 规则目录（如不存在）
sudo mkdir -p /etc/udev/rules.d

# 运行检测工具
sudo python3 checker3

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 验证端口映射
ls -l /dev/rplidar /dev/arm /dev/xbot /dev/sensor
```

## 输出说明

| 设备 | 端口映射 | 检测依据 |
|------|----------|----------|
| 激光雷达 | `/dev/rplidar` | 串口标签包含 `CP2102` |
| 机械臂 | `/dev/arm` | 串口标签包含 `Serial Cable` |
| PCB1 | `/dev/xbot` | 数据帧头部 `aa5530` |
| PCB2 | `/dev/sensor` | 数据帧头部 `aa5538` |

## 注意事项

- 需要 root 权限运行（访问串口设备和写入 `/etc/udev/rules.d/`）
- 需要安装 Python 依赖：`pip install pyserial`
- 适用于 ROS2 下的 XBot 系列机器人开发套件
