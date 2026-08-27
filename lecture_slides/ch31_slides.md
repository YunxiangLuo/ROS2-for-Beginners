# 第31章 颜色检测与YOLO

---

## 学习目标
- 掌握HSV颜色空间和颜色检测
- 学会YOLOv8模型加载和推理
- 实现ROS2中的YOLO检测节点
- 理解颜色+形状联合检测

---

## HSV颜色空间
- H: 色调 (0-180)
- S: 饱和度 (0-255)
- V: 明度 (0-255)
- 相比RGB: 对光照变化更鲁棒
- 颜色检测首选

---

## HSV颜色范围
| 颜色 | H范围 | S范围 | V范围 |
|------|-------|-------|-------|
| 红色 | 0-10, 170-180 | 50-255 | 50-255 |
| 绿色 | 35-85 | 50-255 | 50-255 |
| 蓝色 | 100-130 | 50-255 | 50-255 |
| 黄色 | 20-35 | 50-255 | 50-255 |

---

## 颜色检测流程
1. BGR → HSV转换
2. cv2.inRange() 创建掩码
3. 形态学操作(开运算+闭运算)
4. 轮廓查找
5. 绘制检测结果

---

## 形态学操作
- 开运算: 去除噪点
  - cv2.MORPH_OPEN
  - 先腐蚀后膨胀
- 闭运算: 填充空洞
  - cv2.MORPH_CLOSE
  - 先膨胀后腐蚀

---

## YOLOv8简介
- You Only Look Once v8
- 实时目标检测
- 支持: 检测/分割/姿态估计
- 预训练: COCO 80类
- 模型系列: n/s/m/l/x

---

## 模型选择
| 模型 | 参数量 | mAP | 速度(CPU) |
|------|--------|-----|-----------|
| YOLOv8n | 3.2M | 37.3 | 最快 |
| YOLOv8s | 11.2M | 44.9 | 较快 |
| YOLOv8m | 25.9M | 50.2 | 中等 |
| YOLOv8l | 43.7M | 52.9 | 慢 |
| YOLOv8x | 68.2M | 53.9 | 最慢 |

---

## YOLO调用方式
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
results = model(image, conf=0.5)
boxes = results[0].boxes
```
- 直接使用PyTorch
- 或导出ONNX加速
- 灵活配置置信度阈值

---

## ONNX推理
```python
import onnxruntime as ort
session = ort.InferenceSession('model.onnx')
outputs = session.run(None, {input_name: input_data})
```
- 无需PyTorch环境
- 推理速度更快
- 适合部署

---

## 检测结果消息
- vision_msgs/Detection2DArray
  - detections: Detection2D列表
  - bbox: 边界框
  - results: 分类结果和置信度
- 标准ROS2接口

---

## 按类别分发检测
```python
if cls in self.per_class_pubs:
    self.per_class_pubs[cls].publish(msg)
```
- 每个类别独立话题
- 下游节点按需订阅
- 减少不必要的数据处理

---

## Marker可视化
- visualization_msgs/Marker
  - CUBE/SPHERE: 3D形状
  - TEXT_VIEW_FACING: 文字标签
- 在RViz2中显示检测结果
- 便于可视化调试

---

## 颜色+形状检测
- 结合HSV颜色和轮廓形状
- 形状识别: 三角形/正方形/圆形
- 用途: 分拣不同颜色形状的物体
- 无需深度学习, 轻量快速

---

## 参数调试
- rqt_reconfigure: 运行时调整HSV阈值
- 保存最佳参数配置
- 适应不同光照条件
- 作为YAML文件加载

---

## 思考
- HSV阈值如何适应光照变化?
- YOLOv8在机器人上的FPS要求?
- 如何训练自定义YOLO模型?
- 颜色检测和YOLO的优缺点?

---

## 总结
- HSV颜色检测简单有效
- YOLOv8提供高性能实时检测
- ONNX部署提高推理效率
- 联合检测提高识别准确度
- 可视化结果便于调试
