# 第30章 图像接口与标定

---

## 学习目标
- 掌握ROS2中相机驱动方法
- 学会cv_bridge图像转换
- 理解相机内参标定原理
- 掌握图像矫正基本操作

---

## ROS2图像话题
- sensor_msgs/Image: 原始图像
  - width, height: 尺寸
  - encoding: 编码格式(rgb8/bgr8/mono8)
  - data: 像素数据
- sensor_msgs/CameraInfo: 相机参数
  - K: 内参矩阵
  - D: 畸变系数

---

## usb_cam驱动
- 支持UVC标准USB摄像头
- 发布 /image_raw 话题
- 参数:
  - video_device: 设备路径
  - image_width/height: 分辨率
  - pixel_format: yuyv/mjpeg
  - camera_frame_id: 坐标系

---

## cv_bridge
- ROS Image ↔ OpenCV图像转换
- 核心函数:
  - imgmsg_to_cv2(): ROS→OpenCV
  - cv2_to_imgmsg(): OpenCV→ROS
- 编码自动转换
- 注意内存管理和释放

---

## 常见图像处理
```python
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150)
blurred = cv2.GaussianBlur(cv_image, (5,5), 0)
```
- 灰度转换
- 边缘检测(Canny)
- 高斯模糊
- 形态学操作

---

## 相机内参标定
- 目的: 获取相机内参和畸变系数
- 方法: 棋盘格标定板
- 工具: camera_calibration
- 参数:
  - fx, fy: 焦距
  - cx, cy: 光心坐标
  - k1, k2, p1, p2, k3: 畸变系数

---

## 标定流程
1. 打印棋盘格标定板
2. 测量方格实际边长
3. 启动相机和标定节点
4. 多角度采集标定板图像
5. 触发标定计算
6. 保存标定结果

---

## 标定板要求
- 棋盘格: 黑白相间
- 内部角点: 如8x6
- 方格大小: 如24.5mm
- 打印后贴在硬板上
- 确保平整无折痕

---

## 标定质量指标
- 重投影误差: < 0.5像素
- 采集样本数: > 15
- 覆盖: 各区域和角度
- 进度条: X/Y/Size/Skew全绿

---

## 畸变矫正
```python
undistorted = cv2.undistort(
    image, camera_matrix, dist_coeffs,
    None, new_camera_matrix
)
```
- 去除镜头畸变
- 矫正后图像更真实
- 提高后续视觉任务精度

---

## 图像话题查看
- rqt_image_view: 实时查看
- ros2 topic echo /image_raw: 元信息
- ros2 topic hz /image_raw: 帧率
- ros2 topic bw /image_raw: 带宽

---

## 发布处理结果
- 创建新的Image话题
- 示例: /image_gray, /image_edges
- 方便其他节点订阅
- 可组合多个处理步骤

---

## 录制和回放
```bash
# 录制
ros2 bag record /image_raw /camera_info

# 回放
ros2 bag play bag_name
```
- 用于离线分析
- 可重复实验
- 节省调试时间

---

## 相机TF配置
- camera_link: 相机坐标系
- 手眼标定后: base_link → camera_link
- 静态TF广播器发布
- 用于坐标变换

---

## rqt_reconfigure
- 运行时调整参数
- 适用于图像处理参数调试
- 实时反馈
- 快速找到最佳参数

---

## 思考
- 为什么需要标定?
- 畸变对抓取精度的影响?
- 如何选择摄像头分辨率?
- 帧率和带宽的关系?

---

## 总结
- usb_cam驱动标准USB摄像头
- cv_bridge连接ROS和OpenCV
- camera_calibration完成内参标定
- 畸变矫正提高图像质量
- 录制回放便于离线分析
