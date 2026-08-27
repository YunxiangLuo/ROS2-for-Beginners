# 第33章 视觉大模型

---

## 学习目标
- 了解VLM在机器人中的应用
- 学会调用VLM API进行图像分析
- 掌握ROS2中VLM服务封装方法
- 理解Prompt Engineering

---

## 什么是VLM?
- Vision Language Model (视觉语言模型)
- 同时理解图像和文本
- 代表模型:
  - GPT-4o / GPT-4o-mini
  - Qwen-VL (通义千问)
  - LLaVA (开源)
  - CLIP (OpenAI)

---

## VLM与传统CV对比
| 特性 | 传统CV | VLM |
|------|--------|-----|
| 训练 | 需大量标注 | 少样本/零样本 |
| 泛化 | 任务特定 | 通用 |
| 推理 | 快速 | 较慢(1-3s) |
| 开销 | 本地运行 | 云端API |
| 灵活性 | 固定输出 | 自然语言输出 |

---

## GPT-4o Vision
- 多模态: 文本+图像输入
- 支持: 场景描述, 物体检测, VQA
- 回复格式: JSON, 自然语言
- 上下文: 可设置system prompt

---

## API调用格式
```python
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "指令"},
        {"role": "user", "content": [
            {"type": "text", "text": "问题"},
            {"type": "image_url", "image_url": {
                "url": "data:image/jpeg;base64,..."
            }}
        ]}
    ],
    response_format={"type": "json_object"}
)
```

---

## 图像编码
```python
ret, jpeg = cv2.imencode('.jpg', image,
                          [cv2.IMWRITE_JPEG_QUALITY, 70])
img_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
data_url = f'data:image/jpeg;base64,{img_b64}'
```
- 压缩减少token消耗
- JPEG质量70%平衡质量和大小
- 缩放至512px以内

---

## ROS2封装VLM节点
- 订阅: /image_raw (图像输入)
- 订阅: /vlm/query (用户查询)
- 发布: /vlm/description (场景描述)
- 发布: /vlm/objects (检测物体)
- 发布: /vlm/status (处理状态)

---

## Prompt Engineering
- System Prompt: 定义角色和输出格式
- 结构化输出: 指定JSON Schema
- 示例: 
  ```
  返回JSON格式:
  {"description": "...", "objects": [...]}
  ```
- 控制: temperature=0.1确保确定性

---

## 场景描述
```json
{
  "description": "桌面上有1个蓝色试剂瓶和1个烧杯",
  "objects": [
    {"name": "试剂瓶", "position": "桌面左侧", "color": "蓝色"},
    {"name": "烧杯", "position": "桌面右侧", "color": "透明"}
  ],
  "potential_risks": ["桌面边缘有物体可能跌落"]
}
```

---

## 任务规划
- 自然语言指令 → 结构化任务序列
- 示例: "去桌子上拿蓝色瓶子"
- 输出: navigate(table) → detect(bottle) → pick(bottle)
- LLM理解语义, 生成可执行计划

---

## API选择
- OpenAI: GPT-4o系列 (最好)
- 通义千问: Qwen-VL (国内可用)
- Ollama: LLaVA (本地免费)
- 考虑: 成本, 延迟, 隐私

---

## 本地部署方案
- Ollama: 一键部署
- 支持的VLM: LLaVA, BakLLaVA
- 优点: 免费, 低延迟, 隐私
- 缺点: 精度低于云端

---

## VLM检测 vs YOLO
- VLM: 零样本, 自然语言描述
- YOLO: 固定类别, 实时(30+FPS)
- 结合使用:
  - YOLO做实时检测
  - VLM做异常场景分析

---

## 延迟优化
- 异步调用: 不阻塞主循环
- 图像压缩: 降低分辨率
- 缓存: 相同场景不重复调用
- 本地部署: 减少网络延迟

---

## 思考
- VLM幻觉问题如何处理?
- 如何验证VLM输出正确性?
- 大模型API成本控制策略?
- VLM+传统视觉的混合系统?

---

## 总结
- VLM提供通用的视觉理解能力
- ROS2封装方便集成到机器人系统
- Prompt Engineering决定输出质量
- 云端API精度高但延迟大
- 本地部署适合实时场景
