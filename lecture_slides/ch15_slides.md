# 第15章 PPT：综合实训 — 机械臂辅助化学实验系统

> 共 17 页

---

## P1 · 标题页
**综合实训** | 第15章 | 2+4课时
机械臂辅助初中化学实验自动化系统

---

## P2 · 项目背景
- 初中化学实验：安全风险高、操作精度要求高
- 融合多模态 LLM、YOLO、AR 标签、MoveIt!
- 五阶段递进式工作流

---

## P3 · 系统工作流

```
输入 → 验证 → 感知 → 定位 → 执行
文本    LLM    YOLO    AR/TF   MoveIt!
```

图 15-1：五阶段递进式工作流

---

## P4 · 技术栈对照

| 阶段 | 功能 | 对应课程 |
|------|------|:--:|
| 输入 | 自然语言配方 | 第14章 |
| 验证 | LLM 配比校验 | 第4章+第14章 |
| 感知 | YOLO+VLM | 第3章+第13章 |
| 定位 | AR标签+TF2 | 第7章+第8章 |
| 执行 | MoveIt! | 第12章 |
| 编排 | Action Server | 第5章+第6章 |

---

## P5 · 题1：LLM 配方验证
**Service 节点** | RecipeValidator

程序 15-1：Client → Server → LLM → JSON 返回

---

## P6 · 题1 核心代码
```python
prompt = "请校验以下化学实验配方..."
resp = client.chat.completions.create(
    model="gpt-4o-mini", messages=[...])
result = json.loads(resp.choices[0].message.content)
```

---

## P7 · 题2：YOLO 试剂瓶检测
**Topic 节点** | BottleDetector

程序 15-2：Camera Image → cv_bridge → YOLO → Detection2DArray

---

## P8 · 题2 数据流
```
/camera/image_raw → BottleDetector → /bottle_detections
                      (YOLOv8)
```

---

## P9 · 题3：VLM 标签识别
**Service + Topic 节点** | LabelReader

程序 15-3：ROI 截图 → base64 → GPT-4o Vision → 标签文字比对

---

## P10 · 题3 核心流程
```
检测框 (x,y,w,h) → cv2.imencode → base64 → VLM API
                                           ↓
                              label_text vs expected_material
```

---

## P11 · 题4：TF 空间定位
**TF2 节点** | BottleLocalizer

程序 15-4：lookup_transform(ar_marker_xxx → base_link)

---

## P12 · 题4 TF 帧设计
```
base_link
  └── ar_marker_hcl     (x=-0.3, y=0.2, z=0.1)
  └── ar_marker_naoh    (x=0.3, y=-0.2, z=0.1)
  └── ar_marker_h2o     (x=0.5, y=0.3, z=0.1)
```

---

## P13 · 题5：MoveIt2 抓取规划
**Action Server** | ArmController

程序 15-5：pick → transfer → pour

---

## P14 · 题5 运动流程
```
① Pre-grasp (上方0.1m)
② Grasp (下降抓取)
③ Transfer (移动到试管上方)
④ Pour (倾倒 2秒)
```

---

## P15 · 题6：全流程编排
**Action Server** | ExperimentPipeline

程序 15-6：异步调用链 → 组分循环迭代

---

## P16 · 题6 编排逻辑
```
for each component:
    validate → detect → verify → localize → pick_and_place
    publish_progress()
    if cancelled → cleanup
```

---

## P17 · 本章要点
1. 综合运用 14 章知识完成端到端自动化系统
2. LLM+VLM+YOLO 多模态 AI 栈集成
3. Action Server 编排异步/同步混合工作流
4. TF2+MoveIt2 协同实现精确定位与操作
