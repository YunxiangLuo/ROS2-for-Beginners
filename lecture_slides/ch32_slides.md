# 第32章 AR标签与手眼标定

---

## 学习目标
- 掌握ArUco/AprilTag生成和检测
- 学会标签位姿估计方法
- 理解手眼标定原理
- 完成眼在手外标定

---

## AR标签简介
- 视觉基准标记
- 常见类型:
  - ArUco: OpenCV内置支持
  - AprilTag: 更高精度
  - ARToolKit: 经典方案
- 用途: 定位, 标定, AR应用

---

## ArUco标签
- DICT_4X4_50/100/250
- DICT_5X5_50/100/250
- DICT_6X6_50/250
- DICT_7X7_50/100/250
- 数字: bits大小 + 编码数量
- 更大bits: 更鲁棒

---

## AprilTag
- 家族: 16h5, 25h9, 36h10, 36h11
- 优点:
  - 更低误检率
  - 更高精度
  - 更好的遮挡适应
- OpenCV 4.6+支持

---

## 标签生成
```python
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
image = aruco.generateImageMarker(dictionary, marker_id, 200)
```
- 指定字典和ID
- 输出像素尺寸(如200x200)
- 打印时注意实际尺寸

---

## 标签检测流程
1. 灰度转换
2. detectMarkers(): 检测标签
3. estimatePoseSingleMarkers(): 位姿估计
4. drawDetectedMarkers() + drawAxis()
5. 发布检测结果

---

## 位姿估计原理
- PnP (Perspective-n-Point) 求解
- 已知: 2D角点 + 3D模型点 + 相机内参
- 求解: 旋转向量 + 平移向量
- 需要: 准确的标签尺寸

---

## 检测精度因素
- 标签尺寸: 越大越精确
- 相机分辨率: 越高越好
- 光照条件: 均匀照明
- 遮挡: 避免遮挡
- 畸变: 预先标定矫正

---

## TF广播
- 将检测到的标签位姿发布为TF
- child_frame_id: aruco_1, aruco_2...
- parent_frame_id: camera_link
- 其他节点可通过TF查询

---

## 手眼标定
- 建立相机和机器人的坐标关系
- 眼在手外(Eye-on-Hand):
  - 相机固定, 标定板在末端
  - 求解: base_link → camera_link
- 眼在手上(Eye-in-Hand):
  - 相机在末端, 标定板固定
  - 求解: gripper → camera_link

---

## easy_handeye
- 自动化手眼标定工具
- 支持: ROS 2 Jazzy（具体可用性取决于发行版包）
- 流程:
  1. 移动机械臂到多个位姿
  2. 记录末端位姿 + ArUco位姿
  3. 求解手眼变换矩阵
- 采样: 通常17个点

---

## 标定采样策略
- 覆盖机械臂工作空间
- 保持标定板在视野内
- 姿态多样化
- 避免退化运动(如纯平移)
- 至少12-17个有效样本

---

## 标定结果验证
```bash
# 查看标定变换
ros2 run tf2_ros tf2_echo base_link camera_link

# 验证: 放置标签在桌面
# 检测到的位姿应准确反映桌面位置
```

---

## 保存和应用标定
- 结果保存为YAML文件
- 启动时加载标定参数
- 发布静态TF
- 用于后续视觉抓取

---

## 坐标系变换
- base_link: 机器人基座
- camera_link: 相机
- aruco_marker: 标签
- gripper_centor_link: 末端
- lookup_transform: 任意坐标变换

---

## 思考
- ArUco vs AprilTag, 哪个更精确?
- 为什么需要多样化的采样位姿?
- 标定误差如何传递到抓取精度?
- 如何在线修正标定结果?

---

## 总结
- ArUco标签提供稳定视觉特征
- estimatePoseSingleMarkers得到6D位姿
- easy_handeye自动化手眼标定
- TF协调所有坐标系关系
- 标定精度直接影响抓取成功率
