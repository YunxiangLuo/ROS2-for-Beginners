# 第34章 视觉抓取应用

---

## 学习目标
- 整合视觉和运动规划模块
- 实现完整的视觉抓取Pipeline
- 掌握ArUco引导的自动抓取
- 理解多模块协同工作机制

---

## 视觉抓取系统架构
- 相机: 获取图像
- 视觉检测: 识别目标(ArUco/YOLO)
- TF变换: 目标位姿转换到机器人坐标系
- MoveIt2: 运动规划
- 抓取执行: 手爪控制

---

## Pipeline流程
1. 相机采集图像
2. 检测ArUco标签
3. 估计标签位姿(相机坐标系)
4. TF变换→ 机器人基坐标系
5. 目标位姿发布
6. MoveIt2 IK求解
7. 机械臂运动到目标
8. 闭合手爪抓取
9. 提起 → 移动 → 放置

---

## 坐标变换链
```
camera_link ← 检测位姿
base_link ← 手眼标定变换
target_in_base = T_base_camera * target_in_camera
```
- lookup_transform: 查询变换
- do_transform_pose: 应用变换
- 所有位姿统一到base_link

---

## 服务接口设计
```python
# 触发抓取
ros2 service call /vision_grasp/trigger std_srvs/srv/Trigger

# 返回: {"success": true, "message": "抓取完成"}
```
- 异步执行: 不阻塞客户端
- 状态监控: 通过日志和话题

---

## 抓取策略优化
- 预抓取: 从目标上方接近
- 抓取: 精确到达目标
- 提升: 垂直提起
- 考虑: 目标朝向, 夹爪姿态

---

## 多标签处理
- 同一场景多个ArUco标签
- 通过ID区分目标
- 选择最近的/指定ID的标签
- 顺序处理多个目标

---

## 错误处理
```python
if self.latest_pose is None:
    response.success = False
    response.message = '未检测到标签'
    return response
```
- 无检测: 提示用户放置标签
- 规划失败: 重试/调整策略
- 执行错误: 安全停止

---

## 服务端实现
- Node作为服务端
- 触发后启动线程执行
- 发布目标位姿用于可视化
- 日志记录每一步状态

---

## 视觉测试工具
```python
# 简化版: 仅检测和发布位姿
ros2 run ch34_vision_grasp simple_vision_test
ros2 topic echo /vision/test_pose
```
- 无需MoveIt2即可测试视觉
- 验证检测和变换准确性

---

## 参数调优
- marker_size: 标签实际尺寸
- 光源: 影响检测
- 相机曝光: 自动/手动
- 检测频率: 控制计算负载

---

## 集成步骤
1. 确保手眼标定完成
2. 启动MoveIt2 demo
3. 启动相机
4. 启动视觉抓取Pipeline
5. 放置标签在视野内
6. 触发抓取

---

## 性能分析
| 阶段 | 耗时 |
|------|------|
| 图像获取 | 33ms (30fps) |
| ArUco检测 | 5-10ms |
| 位姿估计 | 2ms |
| TF查询 | 1ms |
| MoveIt2规划 | 100-500ms |
| 轨迹执行 | 2-5s |

---

## 思考
- 如何提高抓取的成功率?
- 深度相机对抓取的改善?
- 移动目标如何实时跟踪?
- 多机械臂协同抓取?

---

## 总结
- 视觉抓取整合视觉+运动规划
- ArUco提供可靠的视觉特征
- TF变换确保坐标系统一
- 服务接口便于系统集成
- 参数调优和错误处理保障可靠性
