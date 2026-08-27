# 第33章 视觉大模型与ROS2应用

## 学习目标
- 了解主流的视觉大模型发展现状
- 掌握GPT-4V、SAM、CLIP等模型的原理与应用
- 学会在ROS2中集成视觉大模型
- 掌握零样本物体检测的实现方法
- 理解场景理解的实现技术

## 33.1 视觉大模型概述

### 33.1.1 大模型发展简史

视觉大模型（Vision-Language Model, VLM）是近年来人工智能领域最重大的突破之一。这些模型通过海量图像-文本对的训练，实现了强大的视觉理解和推理能力。

主要视觉大模型：

| 模型 | 发布时间 | 特点 | 适用场景 |
|------|---------|------|---------|
| CLIP | 2021 | 图文匹配，零样本分类 | 物体识别 |
| SAM | 2023 | 精准分割一切 | 物体分割 |
| GPT-4V | 2023 | 多模态理解推理 | 场景理解 |
| DINOv2 | 2023 | 自监督视觉特征 | 特征提取 |
| Grounding DINO | 2023 | 开放词汇检测 | 零样本检测 |
| LLaVA | 2024 | 开源VLM | 多模态对话 |

### 33.1.2 GPT-4V

GPT-4V（GPT-4 with Vision）是OpenAI的多模态大模型，能够同时理解图像和文本输入。

核心能力：
- 图像内容理解和描述
- 物体检测和定位
- 场景分析和推理
- 文本识别（OCR）
- 空间关系理解

### 33.1.3 SAM (Segment Anything Model)

SAM是Meta AI发布的通用分割模型，可以对图像中的任何物体进行分割。

特点：
- 零样本分割：无需额外训练
- 提示分割：点、框、文本
- 全图分割：自动生成所有掩码
- 实时性：轻量版可在边缘设备运行

### 33.1.4 CLIP (Contrastive Language-Image Pre-training)

CLIP是OpenAI发布的图文对比学习模型，将图像和文本映射到同一语义空间。

应用：
- 零样本图像分类
- 图文检索
- 图像特征提取
- 多模态理解

## 33.2 ROS2与GPT-4V集成

### 33.2.1 GPT-4V场景理解节点

```python
#!/usr/bin/env python3
"""GPT-4V场景理解ROS2节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import openai
import base64
import cv2
import json
import numpy as np


class GPT4VSceneUnderstander(Node):
    def __init__(self):
        super().__init__('gpt4v_scene_understander')

        # 配置参数
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'gpt-4o')
        self.declare_parameter('query_topic', '/vlm/query')
        self.declare_parameter('system_prompt', '你是一个机器人视觉助手。请描述场景中的物体及其相对位置。')

        api_key = self.get_parameter('api_key').value
        self.model = self.get_parameter('model').value
        self.system_prompt = self.get_parameter('system_prompt').value

        # 初始化OpenAI客户端
        self.client = openai.OpenAI(api_key=api_key)
        self.bridge = CvBridge()

        # 缓存最新图像
        self.latest_image = None
        self.latest_query = '请描述当前场景中有什么物体。'

        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_cb, 10
        )
        self.query_sub = self.create_subscription(
            String, '/vlm/query', self.query_cb, 10
        )

        # 发布
        self.description_pub = self.create_publisher(
            String, '/vlm/description', 10
        )
        self.objects_pub = self.create_publisher(
            String, '/vlm/objects', 10
        )

        self.get_logger().info('GPT-4V场景理解节点已启动')

    def image_cb(self, msg: Image):
        """缓存最新图像"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def query_cb(self, msg: String):
        """收到查询请求，触发VLM推理"""
        self.latest_query = msg.data
        self.get_logger().info(f'收到VLM查询: {msg.data}')
        if self.latest_image is not None:
            self.analyze_scene()

    def analyze_scene(self):
        """调用VLM分析场景"""
        # 图像编码为base64
        ret, jpeg = cv2.imencode(
            '.jpg', self.latest_image,
            [cv2.IMWRITE_JPEG_QUALITY, 85]
        )
        img_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": self.system_prompt
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": self.latest_query + "\n请用JSON格式回复: {\"description\": \"场景描述\", \"objects\": [{\"name\": \"物体名\", \"position\": \"相对位置\", \"color\": \"颜色\", \"confidence\": 0.9}]}"
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{img_b64}",
                                    "detail": "low"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=500,
                response_format={"type": "json_object"},
            )

            result = json.loads(
                response.choices[0].message.content
            )

            self.get_logger().info(
                f'场景: {result.get("description", "")[:80]}...'
            )

            # 发布描述
            self.description_pub.publish(
                String(data=result.get('description', ''))
            )
            # 发布物体列表
            self.objects_pub.publish(
                String(data=json.dumps(
                    result.get('objects', []),
                    ensure_ascii=False
                ))
            )

        except Exception as e:
            self.get_logger().error(f'VLM调用失败: {e}')

    def spatial_query(self, question: str) -> dict:
        """空间推理示例查询"""
        if self.latest_image is None:
            return {"error": "无可用图像"}

        ret, jpeg = cv2.imencode(
            '.jpg', self.latest_image,
            [cv2.IMWRITE_JPEG_QUALITY, 85]
        )
        img_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": f"请回答: {question}\n返回JSON: {{\"answer\": \"回答\", \"confidence\": 0.9}}"},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}}
                    ]
                }],
                response_format={"type": "json_object"},
            )
            return json.loads(response.choices[0].message.content)
        except Exception as e:
            return {"error": str(e)}


def main(args=None):
    rclpy.init(args=args)
    node = GPT4VSceneUnderstander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 33.2.2 多后端支持

支持多种VLM后端（OpenAI、通义千问、本地模型）：

```python
class VLMBackend:
    """统一的VLM后端抽象"""

    @staticmethod
    def create_backend(provider: str, **kwargs):
        """工厂方法创建VLM后端"""
        if provider == 'openai':
            return OpenAIVLMBackend(**kwargs)
        elif provider == 'qwen':
            return QwenVLMBackend(**kwargs)
        elif provider == 'ollama':
            return OllamaVLMBackend(**kwargs)
        else:
            raise ValueError(f'不支持的VLM后端: {provider}')


class OpenAIVLMBackend:
    def __init__(self, api_key: str = '', model: str = 'gpt-4o'):
        import openai
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model

    def analyze(self, image_b64: str, query: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{image_b64}"
                    }}
                ]
            }]
        )
        return response.choices[0].message.content


class QwenVLMBackend:
    def __init__(self, api_key: str = '', model: str = 'qwen-vl-max'):
        import openai
        self.client = openai.OpenAI(
            api_key=api_key or os.getenv('DASHSCOPE_API_KEY'),
            base_url='https://dashscope.aliyuncs.com/compatible-mode/v1'
        )
        self.model = model

    def analyze(self, image_b64: str, query: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{image_b64}"
                    }}
                ]
            }]
        )
        return response.choices[0].message.content


class OllamaVLMBackend:
    """本地Ollama VLM模型"""
    def __init__(self, model: str = 'llama3.2-vision:11b'):
        import openai
        self.client = openai.OpenAI(
            base_url='http://localhost:11434/v1',
            api_key='ollama'
        )
        self.model = model

    def analyze(self, image_b64: str, query: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{image_b64}"
                    }}
                ]
            }]
        )
        return response.choices[0].message.content
```

## 33.3 SAM图像分割

### 33.3.1 SAM模型加载

```bash
# 安装SAM
pip install git+https://github.com/facebookresearch/segment-anything.git

# 下载模型权重
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

### 33.3.2 ROS2 SAM分割节点

```python
#!/usr/bin/env python3
"""SAM图像分割ROS2节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch


class SAMSegmenter(Node):
    def __init__(self):
        super().__init__('sam_segmenter')
        self.bridge = CvBridge()

        # 加载SAM模型
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'使用设备: {self.device}')

        try:
            from segment_anything import sam_model_registry, SamPredictor
            sam = sam_model_registry['vit_h'](
                checkpoint='sam_vit_h_4b8939.pth'
            )
            sam.to(device=self.device)
            self.predictor = SamPredictor(sam)
            self.get_logger().info('SAM模型加载成功')
        except Exception as e:
            self.get_logger().error(f'SAM加载失败: {e}')
            self.predictor = None

        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.mask_pub = self.create_publisher(
            Image, '/sam/masks', 10
        )
        self.vis_pub = self.create_publisher(
            Image, '/sam/visualization', 10
        )

    def image_callback(self, msg):
        if self.predictor is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # SAM推理
        self.predictor.set_image(rgb)

        # 自动全图分割（点网格提示）
        h, w = frame.shape[:2]
        points = []
        for y in range(0, h, 50):
            for x in range(0, w, 50):
                points.append([x, y])

        if len(points) == 0:
            return

        point_coords = np.array(points)
        point_labels = np.ones(len(points))

        masks, scores, _ = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=False
        )

        # 生成可视化
        vis = frame.copy()
        combined_mask = np.zeros((h, w), dtype=np.uint8)

        for i, mask in enumerate(masks):
            if scores[i] < 0.5:
                continue
            mask_bool = mask.astype(bool)
            combined_mask[mask_bool] = 255

            # 随机颜色
            color = np.random.randint(0, 255, 3).tolist()
            vis[mask_bool] = vis[mask_bool] * 0.5 + \
                             np.array(color) * 0.5

        # 发布结果
        mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        vis_msg = self.bridge.cv2_to_imgmsg(
            vis.astype(np.uint8), 'bgr8'
        )
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)

        cv2.imshow('SAM Segmentation', vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SAMSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 33.3.3 SAM提示分割

```python
class SAMPromptSegmenter(Node):
    """基于提示的SAM分割"""

    def segment_with_box(self, image, bbox):
        """
        使用矩形框提示分割
        bbox: [x1, y1, x2, y2]
        """
        self.predictor.set_image(image)
        bbox_input = np.array([bbox])
        masks, scores, _ = self.predictor.predict(
            box=bbox_input,
            multimask_output=False
        )
        return masks[0], scores[0]

    def segment_with_point(self, image, point_x, point_y):
        """
        使用点提示分割
        """
        self.predictor.set_image(image)
        masks, scores, _ = self.predictor.predict(
            point_coords=np.array([[point_x, point_y]]),
            point_labels=np.array([1]),
            multimask_output=True
        )
        # 返回最高得分的掩码
        best_idx = np.argmax(scores)
        return masks[best_idx], scores[best_idx]

    def segment_with_text(self, image, text):
        """
        使用文本提示分割（需要Grounding DINO + SAM）
        """
        # 先使用文本检测获取bbox
        bboxes = self.grounded_dino_detect(image, text)
        masks = []
        for bbox in bboxes:
            mask, score = self.segment_with_box(image, bbox)
            masks.append((mask, score, bbox))
        return masks
```

## 33.4 CLIP零样本分类

### 33.4.1 CLIP模型用法

```bash
# 安装CLIP
pip install git+https://github.com/openai/CLIP.git
```

### 33.4.2 ROS2 CLIP零样本检测节点

```python
#!/usr/bin/env python3
"""CLIP零样本分类ROS2节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
import clip
import numpy as np


class CLIPZeroShot(Node):
    def __init__(self):
        super().__init__('clip_zeroshot')

        # 可配置的类别列表
        self.declare_parameter('classes', [
            'robot', 'person', 'table', 'chair',
            'bottle', 'book', 'box', 'tool'
        ])
        self.classes = self.get_parameter('classes').value

        self.bridge = CvBridge()

        # 加载CLIP模型
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model, self.preprocess = clip.load(
            'ViT-B/32', device=self.device
        )
        self.get_logger().info(
            f'CLIP模型加载成功，设备: {self.device}'
        )

        # 编码类别文本
        self.encode_classes()

        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.result_pub = self.create_publisher(
            String, '/clip/classification', 10
        )

    def encode_classes(self):
        """编码类别文本为特征向量"""
        text_inputs = torch.cat([
            clip.tokenize(f'a photo of a {c}' for c in self.classes)
        ]).to(self.device)

        with torch.no_grad():
            self.text_features = self.model.encode_text(text_inputs)
            self.text_features /= self.text_features.norm(
                dim=-1, keepdim=True
            )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 预处理图像
        image_input = self.preprocess(
            Image.fromarray(rgb)
        ).unsqueeze(0).to(self.device)

        # 编码图像
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            image_features /= image_features.norm(
                dim=-1, keepdim=True
            )

            # 计算相似度
            similarity = (100.0 * image_features @ self.text_features.T)
            similarity = similarity.softmax(dim=-1)

        # 获取预测结果
        probs = similarity.cpu().numpy()[0]
        top_idx = np.argsort(probs)[::-1][:3]

        result = {
            'predictions': [
                {
                    'class': self.classes[idx],
                    'confidence': float(probs[idx])
                }
                for idx in top_idx
            ]
        }

        # 发布结果
        import json
        self.result_pub.publish(
            String(data=json.dumps(result, ensure_ascii=False))
        )

        # 显示结果
        top_class = self.classes[top_idx[0]]
        top_prob = probs[top_idx[0]]
        cv2.putText(
            frame,
            f'{top_class}: {top_prob:.2%}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1, (0, 255, 0), 2
        )
        cv2.imshow('CLIP Zero-shot', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CLIPZeroShot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 33.5 开放词汇检测（Grounding DINO）

### 33.5.1 Grounding DINO集成

```bash
# 安装Grounding DINO
pip install groundingdino-py
```

```python
class GroundingDINODetector(Node):
    """开放词汇目标检测"""

    def __init__(self):
        super().__init__('grounding_dino_detector')
        self.bridge = CvBridge()

        from groundingdino.models import build_model
        from groundingdino.util import get_tokenizer

        # 加载模型
        self.model = build_model(
            config='GroundingDINO/config/GroundingDINO_SwinT_OGC.py'
        )
        self.tokenizer = get_tokenizer()

        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            String, '/grounding_dino/detections', 10
        )

    def detect_with_text(self, image, text_prompt):
        """
        文本提示检测
        例如: detect_with_text(image, 'bottle. cup. box.')
        """
        # 模型推理
        boxes, logits, phrases = self.model.predict(
            image, text_prompt, box_threshold=0.3, text_threshold=0.25
        )

        detections = []
        for box, logit, phrase in zip(boxes, logits, phrases):
            detections.append({
                'bbox': box.tolist(),
                'confidence': float(logit),
                'label': phrase,
            })

        return detections

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = self.detect_with_text(
            frame,
            'bottle. cup. book. box. person. chair. table.'
        )
        import json
        self.detection_pub.publish(
            String(data=json.dumps(detections, ensure_ascii=False))
        )
```

## 33.6 综合应用：视觉大模型机器人

### 33.6.1 完整系统架构

```
┌─────────────────────────────────────────────┐
│             具身智能机器人系统                 │
│                                                │
│  ┌──────────┐   自然语言指令                    │
│  │  用户输入  │─────────┐                     │
│  └──────────┘         │                     │
│                        ▼                     │
│  ┌──────────────────────────────┐           │
│  │     LLM任务规划器             │           │
│  │  (GPT-4/Qwen/Ollama)        │           │
│  └──────────┬───────────────────┘           │
│             │ JSON任务序列                    │
│  ┌──────────▼───────────────────┐           │
│  │     动作转换器                │           │
│  │  (自然语言→ROS2 Action)      │           │
│  └──┬────────┬────────┬────────┘             │
│     │        │        │                      │
│  ┌──▼──┐  ┌──▼──┐  ┌──▼──────┐             │
│  │Nav2 │  │YOLO │  │MoveIt2  │             │
│  │导航 │  │检测 │  │机械臂   │             │
│  └─────┘  └──┬───┘  └────────┘             │
│              │                               │
│  ┌───────────▼──────────────────┐           │
│  │    VLM场景理解               │           │
│  │  (GPT-4V/SAM/CLIP)          │           │
│  └──────────────────────────────┘           │
└─────────────────────────────────────────────┘
```

### 33.6.2 端到端具身智能体

```python
class EmbodiedAgent(Node):
    """
    端到端具身智能体
    整合 LLM + VLM + 机器人控制
    """
    def __init__(self):
        super().__init__('embodied_agent')

        self.states = ['idle', 'observing', 'planning', 'executing']
        self.current_state = 'idle'

        self.memory = {
            'observations': [],
            'object_locations': {},
            'task_history': [],
        }

        # 初始化各模块
        self.vlm = self.init_vlm()
        self.llm = self.init_llm()

        self.get_logger().info('具身智能体已初始化')

    def init_vlm(self):
        """初始化VLM后端"""
        return VLMBackend.create_backend('openai')

    def init_llm(self):
        """初始化LLM后端"""
        return LLMBackend.create_backend('openai')

    def perceive(self, image):
        """感知环境"""
        description = self.vlm.analyze(
            image,
            "请详细描述场景中的所有物体、位置和状态。"
        )
        self.memory['observations'].append(description)
        return description

    def plan(self, task, environment):
        """生成任务计划"""
        prompt = f"""
当前任务: {task}
环境描述: {environment}
可用操作: navigate, detect, pick, place, wait
请生成执行计划，返回JSON格式。
"""
        plan = self.llm.generate(prompt)
        return plan

    def execute_step(self, step):
        """执行单步动作"""
        action = step['action']
        if action == 'navigate':
            return self.execute_navigation(step['params'])
        elif action == 'detect':
            return self.execute_detection(step['params'])
        elif action == 'pick':
            return self.execute_pick(step['params'])
        elif action == 'place':
            return self.execute_place(step['params'])
        return False


class LLMBackend:
    """统一的LLM后端抽象"""
    @staticmethod
    def create_backend(provider, **kwargs):
        if provider == 'openai':
            return OpenAILLMBackend(**kwargs)
        return OpenAILLMBackend(**kwargs)


class OpenAILLMBackend:
    def __init__(self, model='gpt-4o-mini'):
        import openai
        self.client = openai.OpenAI()
        self.model = model

    def generate(self, prompt):
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"},
        )
        import json
        return json.loads(response.choices[0].message.content)
```

## 课后练习

1. 编写ROS2节点，集成GPT-4V或通义千问VL模型，对相机图像进行场景描述。

2. 使用SAM模型对图像中的物体进行分割，并将分割掩码发布为ROS2图像话题。

3. 使用CLIP模型实现零样本物体分类，识别桌面上5种不同物体（如瓶子、书、手机、笔、杯子）。

4. 编写节点，接收自然语言查询（如"桌子左边有什么？"），使用VLM分析图像并返回答案。

5. 设计一个完整系统：VLM感知环境 → LLM规划任务序列 → 机器人执行动作。
