# 第15章 实验指导书：综合实训

## 当前仓库仿真验证：感知端与 xArm 操作端分层联调

### 实验目标

用移动机器人仿真提供图像/内参，用 xArm 仿真提供 MoveIt2 规划环境，练习综合系统中的消息、TF 和任务接口分层。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=true rviz:=true drive:=false
ros2 topic echo /camera/camera_info --once
```

在已安装兼容 `xarm_description` 2.0.0 的环境中另开终端：

```bash
source /path/to/xarm_description_workspace/install/setup.bash
ros2 launch xarm_ros2_arm_only arm_only.launch.py
```

### 观察与验收

感知端检查 `/camera/image_raw`、`/camera/camera_info` 和 TF；操作端在 RViz 检查 `xarm` 规划组和轨迹。源码：`src/robot_sim_demo/`、`src/xarm/`、`src/lab_code/ch15_lab/`。当前不宣称完成真实化学试剂识别或自动抓取。

> **实验课时**：4 课时（180 分钟）  
> **实验平台**：XBot-U Gazebo 仿真 + OpenAI/Qwen API  

---

## 实验目标

完成 6 个综合编程题目，构建完整的**机械臂辅助化学实验自动化系统**。

---

## 题目 1：LLM 配方验证服务（约 30 分钟）

### 要求
编写 `recipe_validator` 服务节点，实现化学实验配方的 LLM 自动校验。

### 实现要点
1. 定义 `RecipeValidate.srv`（接收配方文本，返回校验结果）
2. 构建 LLM Prompt：包含配比计算规则和安全检查项
3. 解析 LLM 返回的 JSON 结果

### 测试用例
```
输入配方：
"取 5ml 稀盐酸 (HCl, 1mol/L) 加入试管，
 再加入 5ml 氢氧化钠溶液 (NaOH, 1mol/L)，
 滴入 2 滴酚酞指示剂，观察颜色变化"

期望输出：
{
  "is_valid": true,
  "feedback": "配比正确：HCl + NaOH → NaCl + H₂O，
               1:1 摩尔比，产物中性",
  "products": ["NaCl", "H₂O"],
  "safety_warnings": ["稀盐酸具腐蚀性，需佩戴手套"]
}
```

### 验收标准
- ✓ 正确调用 LLM API
- ✓ 返回结构化 JSON
- ✓ 校验配比逻辑
- ✓ 识别安全隐患

### 参考代码
`lab_code/ch15_lab/src/recipe_validator.py`

---

## 题目 2：YOLO 试剂瓶检测（约 30 分钟）

### 要求
订阅 XBot-U 相机话题 `/camera/image_raw`，使用 YOLOv8 实时检测实验台上的试剂瓶。

### 实现要点
1. `cv_bridge` 将 ROS Image 转为 OpenCV 格式
2. 加载 YOLOv8n 预训练模型（或自定义训练的试剂瓶模型）
3. 发布 `/bottle_detections` 话题（Detection2DArray）

### 测试方法
```bash
# 终端1：启动 Gazebo 仿真
ros2 launch robot_sim_demo_ros2 sim_bringup.launch.py \
  use_gazebo:=true gz_headless:=false

# 终端2：启动检测节点
ros2 run bottle_detector bottle_detector

# 终端3：查看检测结果
ros2 topic echo /bottle_detections
```

### 验收标准
- ✓ 相机图像正确转换为 OpenCV 格式
- ✓ YOLO 模型成功加载并推理
- ✓ 检测结果话题正常发布
- ✓ `ros2 topic echo` 可查看检测结果

### 参考代码
`lab_code/ch15_lab/src/bottle_detector.py`

---

## 题目 3：VLM 标签文字识别（约 30 分钟）

### 要求
基于题2的检测框截取试剂瓶图像区域，发送给 VLM（GPT-4o-Vision）读取标签文字，与配方期望值比对。

### 实现要点
1. 订阅 `/bottle_detections` 获取检测框坐标
2. 从原始图像中截取 ROI 区域
3. 将 ROI 编码为 base64 → 发送到 GPT-4o API
4. 解析 VLM 识别结果，与期望材料名称比对

### 测试方法
```bash
# 终端1：Gazebo 仿真运行中
# 终端2：YOLO 检测节点运行中
# 终端3：启动标签识别
ros2 run label_reader label_reader --ros-args \
  -p expected_material:="HCl"
```

### 验收标准
- ✓ 正确截取检测框对应的图像区域
- ✓ VLM 成功识别标签文字
- ✓ 比对结果：matches_expected = True/False
- ✓ 输出 confidence 置信度

### 参考代码
`lab_code/ch15_lab/src/label_reader.py`

---

## 题目 4：TF 空间定位（约 30 分钟）

### 要求
在 Gazebo 世界中添加 AR 标签虚拟标记，通过 TF2 查询试剂瓶的 3D 空间位姿。

### 实现要点
1. 在 Gazebo 仿真中添加 4 个带有 frame_id 的标记模型
2. 广播 TF 变换（ar_marker_hcl → base_link 等）
3. 编写 `lookup_transform()` 查询代码
4. 发布 PoseStamped 话题（用于 MoveIt2 抓取）

### 标记 frame 清单
| frame_id | 位置 (x,y,z) | 对应试剂 |
|----------|-------------|---------|
| ar_marker_hcl | (-0.3, 0.2, 0.1) | 稀盐酸 |
| ar_marker_naoh | (0.3, -0.2, 0.1) | 氢氧化钠 |
| ar_marker_h2o | (0.5, 0.3, 0.1) | 蒸馏水 |
| ar_marker_phenolphthalein | (-0.5, 0.0, 0.1) | 酚酞指示剂 |

### 验收标准
- ✓ TF 广播正常工作
- ✓ `lookup_transform` 成功查询 4 个 marker 位姿
- ✓ `ros2 run tf2_tools view_frames` 生成 TF 树

### 参考代码
`lab_code/ch15_lab/src/bottle_localizer.py`

---

## 题目 5：MoveIt2 抓取规划（约 30 分钟）

### 要求
基于题4的 3D 位姿结果，使用 MoveItPy 编写机械臂 pick → transfer → pour 完整流程。

### 实现要点
1. 初始化 MoveItPy 规划组
2. 编写 `plan_pick(pre_grasp_pose)` 方法
3. 编写 `plan_transfer(from_pose, to_pose)` 方法
4. 编写 `plan_pour(duration_sec)` 倾倒动作
5. 使用 Action Server 对外暴露接口

### 运动流程伪代码
```python
# 1. 预抓取（目标上方 0.1m）
move_to(target_pose, z_offset=0.1)
# 2. 抓取（下降到目标）
move_to(target_pose)
close_gripper()
# 3. 转移（到试管上方）
move_to(pour_pose)
# 4. 倾倒（末端旋转 180°，保持 2 秒）
rotate_end_effector(180)
sleep(2.0)
open_gripper()
```

### 验收标准
- ✓ MoveItPy 初始化成功
- ✓ plan_and_execute 生成有效轨迹
- ✓ 3 阶段运动流程完整
- ✓ 碰撞检测正常工作

### 参考代码
`lab_code/ch15_lab/src/arm_controller.py`

---

## 题目 6：全流程编排（约 30 分钟）

### 要求
编写 Action Server `ExperimentPipeline`，编排题1-5的所有模块，按配方组分列表循环迭代完成全自动化实验。

### 实现要点
1. 定义 `ExperimentPipeline.action`
2. 创建各模块的 ROS 2 客户端
3. 异步调用链：validate → detect → verify → localize → pick_and_place
4. 发布实时进度反馈（current_step / total_steps）
5. 支持取消操作和错误恢复

### 编排伪代码
```python
async def execute(self, goal_handle):
    recipe = goal_handle.request.recipe_text
    # 步骤1-2：校验+解析
    valid = await self.validate_recipe(recipe)
    components = await self.parse_components(recipe)
    # 步骤3-5：循环处理每个组分
    for comp in components:
        detection = await self.detect_bottle(comp.name)
        verified = await self.verify_label(detection, comp.name)
        pose = await self.localize_bottle(comp.name)
        await self.pick_and_place(pose, comp.name)
    # 完成
    goal_handle.succeed()
```

### 测试方法
```bash
# 启动全流程编排
ros2 run experiment_pipeline pipeline_server &

# 发送实验配方
ros2 action send_goal /run_experiment experiment_interfaces/action/ExperimentPipeline \
  "{recipe_text: '取5ml HCl加入试管，再加入5ml NaOH...'}" --feedback
```

### 验收标准
- ✓ Action Server 正常启动
- ✓ 6 步流程按序执行
- ✓ 实时反馈 current_step 进度
- ✓ `Ctrl+C` 取消操作正常工作
- ✓ 单个步骤失败时能输出错误信息

### 参考代码
`lab_code/ch15_lab/src/experiment_pipeline.py`

---

## 评分标准

| 题目 | 分值 | 核心考核点 |
|:--:|:--:|------|
| 题1 | 15 | LLM API 调用、结构化输出解析 |
| 题2 | 15 | cv_bridge、YOLO 推理、ROS 话题 |
| 题3 | 15 | VLM 多模态 API、图像编码 |
| 题4 | 15 | TF2 广播/查询、坐标系计算 |
| 题5 | 20 | MoveItPy 规划、运动控制 |
| 题6 | 20 | Action 编排、异步调用链 |
| **合计** | **100** | |
