# 第14章 实验手册: 视觉大模型 + ROS 2

## 当前仓库仿真验证：相机到 VLM mock 服务

### 实验目标

使用 Gazebo 相机作为视觉大模型服务的输入，在无 API Key 时先验证图像订阅、结构化响应和任务话题发布。

### 运行步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=false
```

```bash
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera/camera_info --once
```

使用 `src/lab_code/ch20_lab/` 的 mock 设计接收图像或其摘要，检查服务响应是否为可解析 JSON，再切换真实 provider。

### 观察与边界

相机话题存在即可验收输入链路；mock JSON 不是 VLM 的真实识别结果。源码：`src/robot_sim_demo/`、`src/lab_code/ch20_lab/`。

## 环境说明
本实验使用 XBot-U 仿真环境, 需要 OpenAI API Key (或 Qwen/本地 Ollama), 预装 openai 库。

---

## 练习1: LLM 文本规划节点 (~30 分钟)

### 目标
编写 ROS 2 节点, 调用 LLM API 将自然语言指令解析为结构化任务序列。

### 步骤

#### 1.1 安装依赖和配置 API
```bash
pip install openai

# 设置 API Key (任选一种)
# OpenAI:
export OPENAI_API_KEY="sk-xxx"

# 或 Qwen (通义千问):
export DASHSCOPE_API_KEY="sk-xxx"

# 或本地 Ollama:
ollama serve &
ollama pull qwen2.5:7b
```

#### 1.2 编写 LLM 规划节点
```python
#!/usr/bin/env python3
"""练习1: LLM 任务规划 ROS 2 节点"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import os


class SimpleLLMPlanner(Node):
    """LLM 规划器: 自然语言 → 结构化任务"""
    def __init__(self):
        super().__init__('simple_llm_planner')

        # 参数配置
        self.declare_parameter('api_type', 'openai')  # openai / qwen / ollama
        self.declare_parameter('model', 'gpt-4o-mini')
        self.declare_parameter('temperature', 0.2)

        api_type = self.get_parameter('api_type').value
        model = self.get_parameter('model').value

        # 配置 API 客户端
        if api_type == 'qwen':
            self.client = openai.OpenAI(
                api_key=os.getenv('DASHSCOPE_API_KEY', ''),
                base_url='https://dashscope.aliyuncs.com/compatible-mode/v1'
            )
        elif api_type == 'ollama':
            self.client = openai.OpenAI(
                base_url='http://localhost:11434/v1',
                api_key='ollama'
            )
        else:
            self.client = openai.OpenAI(
                api_key=os.getenv('OPENAI_API_KEY', '')
            )

        self.model = model
        self.temperature = self.get_parameter('temperature').value

        # 订阅自然语言指令
        self.cmd_sub = self.create_subscription(
            String, '/llm/command', self.command_callback, 10
        )
        # 发布任务序列
        self.plan_pub = self.create_publisher(
            String, '/llm/plan', 10
        )
        # 发布执行状态
        self.status_pub = self.create_publisher(
            String, '/llm/status', 10
        )

        # 环境知识库
        self.known_locations = {
            'entrance': (0.0, 0.0, '入口'),
            'shelf_a': (2.0, 2.0, 'A货架'),
            'shelf_b': (-2.0, 2.0, 'B货架'),
            'desk': (3.0, -1.0, '办公桌'),
            'station': (0.0, -3.0, '充电站'),
        }

        self.get_logger().info(f'LLM 规划器已就绪 (模型: {model})')

    def command_callback(self, msg: String):
        """处理自然语言指令"""
        instruction = msg.data.strip()
        self.get_logger().info(f'收到指令: {instruction}')
        self.status_pub.publish(String(data=f'正在分析: {instruction}'))

        # 调用 LLM 规划
        plan = self.plan_task(instruction)

        if plan:
            plan_str = json.dumps(plan, ensure_ascii=False, indent=2)
            self.plan_pub.publish(String(data=plan_str))
            self.status_pub.publish(String(data=f'规划完成: {len(plan.get("tasks", []))} 个任务'))
            self.get_logger().info(f'任务计划:\n{plan_str}')
        else:
            self.status_pub.publish(String(data='规划失败'))

    def plan_task(self, instruction: str) -> dict:
        """调用 LLM 生成任务计划"""
        system_prompt = f"""你是一个机器人任务规划器。将用户的自然语言指令解析为结构化任务序列。

环境信息:
- 10m×10m 仓库场景
- 已知地点和坐标:
{json.dumps({k: {'coord': v[:2], 'desc': v[2]} for k, v in self.known_locations.items()}, indent=2)}

可用的操作:
- navigate(location): 导航到指定地点
- detect(object_class): 检测特定物体类别
- pick(object_name): 抓取物体
- place(location): 放置物体到指定地点
- scan(): 360度扫描周围环境
- wait(seconds): 等待指定秒数
- speak(text): 语音播报

规则:
1. 移动前检查路径安全
2. 未知地点可合理推测坐标 (x:-5~5, y:-5~5)
3. 连续任务间无需重复确认

请仅返回 JSON 格式, 不要包含任何其他文字:
{{"tasks": [{{"step": 1, "action": "动作名", "target": "目标", "params": {{"额外参数": "值"}}}}], "summary": "一句话总结"}}
"""
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": instruction}
                ],
                temperature=self.temperature,
                max_tokens=1000,
            )
            content = response.choices[0].message.content.strip()
            # 清理可能的 markdown 标记
            content = content.replace('```json', '').replace('```', '').strip()
            return json.loads(content)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 解析失败: {e}\n原始回复: {content}')
            return None
        except Exception as e:
            self.get_logger().error(f'LLM 调用失败: {e}')
            return None

    def test_instructions(self):
        """运行测试指令集"""
        test_cases = [
            "去 A 货架拿一个瓶子",
            "从入口出发, 先去 B 货架检查, 然后到办公桌",
            "扫描仓库一圈, 如果发现人就说你好",
            "去充电站待机 5 秒, 然后去入口",
        ]
        for i, instr in enumerate(test_cases):
            self.get_logger().info(f'=== 测试 {i+1}/{len(test_cases)}: {instr} ===')
            plan = self.plan_task(instr)
            if plan:
                self.get_logger().info(f'结果: {json.dumps(plan, ensure_ascii=False)}')
            else:
                self.get_logger().error('规划失败')
            self.get_logger().info('---')


def main():
    rclpy.init()
    node = SimpleLLMPlanner()

    import threading
    # 2秒后自动运行测试
    def run_tests():
        import time
        time.sleep(2)
        node.test_instructions()

    threading.Thread(target=run_tests, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 1.3 运行验证
```bash
# 运行 LLM 规划器 (自动执行测试)
python3 lab14_exercise1.py

# 或通过话题发送自定义指令
ros2 topic pub /llm/command std_msgs/String "data: '去充电站然后到 A 货架拿杯子'"

# 查看规划结果
ros2 topic echo /llm/plan
```

### 验收标准
- [ ] 成功解析自然语言指令为 JSON 任务序列
- [ ] 任务序列包含正确的 action/target 字段
- [ ] 未知地点能合理推测坐标
- [ ] 支持 OpenAI / Qwen / Ollama 至少一种后端

---

## 练习2: VLM 场景描述与 ROS 2 集成 (~30 分钟)

### 目标
实现 VLM 节点分析相机图像, 生成场景描述和物体列表。

### 步骤

#### 2.1 编写 VLM 场景理解节点
```python
#!/usr/bin/env python3
"""练习2: VLM 场景理解 ROS 2 节点"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import openai
import base64
import json
import cv2
import os


class VLMSceneAnalyzer(Node):
    """VLM 场景分析器"""
    def __init__(self):
        super().__init__('vlm_scene_analyzer')

        self.declare_parameter('model', 'gpt-4o')
        self.declare_parameter('api_type', 'openai')

        api_type = self.get_parameter('api_type').value
        model = self.get_parameter('model').value

        # 配置客户端
        if api_type == 'qwen':
            self.client = openai.OpenAI(
                api_key=os.getenv('DASHSCOPE_API_KEY', ''),
                base_url='https://dashscope.aliyuncs.com/compatible-mode/v1'
            )
        else:
            self.client = openai.OpenAI(
                api_key=os.getenv('OPENAI_API_KEY', '')
            )

        self.model = model
        self.bridge = CvBridge()

        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10
        )
        self.query_sub = self.create_subscription(
            String, '/vlm/query', self.query_cb, 10
        )

        # 发布
        self.desc_pub = self.create_publisher(String, '/vlm/description', 10)
        self.obj_pub = self.create_publisher(String, '/vlm/objects', 10)
        self.status_pub = self.create_publisher(String, '/vlm/status', 10)

        self.latest_image = None
        self.get_logger().info('VLM 场景分析器已就绪')

    def image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def query_cb(self, msg: String):
        if self.latest_image is None:
            self.status_pub.publish(String(data='等待图像...'))
            return

        query = msg.data
        self.get_logger().info(f'VLM 查询: {query}')
        self.status_pub.publish(String(data=f'分析中: {query}'))

        result = self.analyze(query)
        if result:
            self.desc_pub.publish(String(
                data=result.get('description', '无描述')
            ))
            self.obj_pub.publish(String(
                data=json.dumps(result.get('objects', []), ensure_ascii=False)
            ))
            self.status_pub.publish(String(data='分析完成'))
        else:
            self.status_pub.publish(String(data='分析失败'))

    def analyze(self, query: str) -> dict:
        """调用 VLM 分析图像"""
        # 图像压缩 + Base64
        h, w = self.latest_image.shape[:2]
        # 缩放到 512px 以减少 token 消耗
        scale = min(512 / max(h, w), 1.0)
        if scale < 1.0:
            img = cv2.resize(self.latest_image, (int(w * scale), int(h * scale)))
        else:
            img = self.latest_image

        ret, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        img_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        data_url = f'data:image/jpeg;base64,{img_b64}'

        # 系统提示
        system_prompt = """你是一个仓库机器人视觉助手。请分析相机图像并回答用户问题。
请用 JSON 回复:
{
  "description": "场景的自然语言描述 (中文)",
  "objects": [
    {"name": "物体名称", "position": "物体的粗略位置 (左/右/中/前/远)", "confidence": 0.9}
  ],
  "potential_risks": ["风险1", "风险2"],
  "answer": "对用户问题的直接回答"
}
"""
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": query},
                            {"type": "image_url", "image_url": {"url": data_url, "detail": "low"}},
                        ]
                    }
                ],
                max_tokens=500,
                response_format={"type": "json_object"},
            )
            content = response.choices[0].message.content.strip()
            return json.loads(content)
        except Exception as e:
            self.get_logger().error(f'VLM 调用失败: {e}')
            return None

    def run_test_queries(self):
        """测试场景分析查询"""
        test_queries = [
            "房间里有几个人? 他们在做什么?",
            "桌上有什么物体? 列出所有可见的物品",
            "有没有哪条路是畅通的可以通行?",
            "最近的门在什么方向?",
        ]

        import threading
        import time

        def send_queries():
            time.sleep(3)  # 等待图像累积
            for q in test_queries:
                self.get_logger().info(f'测试查询: {q}')
                result = self.analyze(q)
                if result:
                    self.get_logger().info(
                        f'场景: {result.get("description", "")[:80]}...'
                    )
                    self.get_logger().info(
                        f'物体: {len(result.get("objects", []))} 个'
                    )
                time.sleep(2)

        threading.Thread(target=send_queries, daemon=True).start()


def main():
    rclpy.init()
    node = VLMSceneAnalyzer()
    node.run_test_queries()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 2.2 运行验证
```bash
# 终端1: 启动相机
ros2 launch xbot_sim xbot_gazebo_camera.launch.py

# 终端2: 运行 VLM 分析器
python3 lab14_exercise2.py

# 终端3: 发送自定义查询
ros2 topic pub /vlm/query std_msgs/String "data: '前方有什么障碍物?'"
ros2 topic echo /vlm/description
```

### 验收标准
- [ ] VLM 成功分析图像并返回 JSON
- [ ] 场景描述合理准确
- [ ] 物体列表包含名称和位置
- [ ] 支持多种查询类型

---

## 练习3: 自然语言指令驱动机器人导航 (~30 分钟)

### 目标
整合 LLM 规划 + Nav2 导航, 实现完整的 "自然语言 → 机器人运动" 管道。

### 步骤

#### 3.1 编写端到端 NL-Navigation 节点
```python
#!/usr/bin/env python3
"""练习3: 自然语言驱动机器人导航"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import openai
import json
import math
import os
import time


class NLNavigation(Node):
    """自然语言 → LLM 规划 → Nav2 导航"""
    def __init__(self):
        super().__init__('nl_navigation')

        # LLM 配置
        self.declare_parameter('model', 'gpt-4o-mini')
        self.client = openai.OpenAI(api_key=os.getenv('OPENAI_API_KEY', ''))
        self.model = self.get_parameter('model').value

        # Nav2 Action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅指令
        self.cmd_sub = self.create_subscription(
            String, '/nl/command', self.command_cb, 10
        )
        # 发布状态
        self.status_pub = self.create_publisher(String, '/nl/status', 10)
        self.plan_pub = self.create_publisher(String, '/nl/plan', 10)

        # 环境知识
        self.locations = {
            '入口': (0.0, 0.0),
            'entrance': (0.0, 0.0),
            'A 货架': (2.0, 2.0),
            'shelf_a': (2.0, 2.0),
            'B 货架': (-2.0, 2.0),
            'shelf_b': (-2.0, 2.0),
            '办公桌': (3.0, -1.0),
            'desk': (3.0, -1.0),
            '充电站': (0.0, -3.0),
            'station': (0.0, -3.0),
        }

        self.get_logger().info('NL-Navigation 节点已就绪')

    def command_cb(self, msg: String):
        """处理自然语言指令"""
        instruction = msg.data.strip()
        self.get_logger().info(f'指令: {instruction}')
        self.status_pub.publish(String(data=f'处理: {instruction}'))

        # 1. LLM 提取目标地点
        targets = self.extract_targets(instruction)
        if not targets:
            self.status_pub.publish(String(data='未识别到目标地点'))
            return

        self.plan_pub.publish(String(data=json.dumps(targets, ensure_ascii=False)))
        self.get_logger().info(f'规划路线: {json.dumps(targets, ensure_ascii=False)}')

        # 2. 依次导航到每个目标点
        for i, target in enumerate(targets):
            location = target['location']
            action = target.get('action', 'navigate')

            if location.lower() in self.locations:
                x, y = self.locations[location.lower()]
                self.get_logger().info(
                    f'[{i+1}/{len(targets)}] 导航到: {location} ({x}, {y})'
                )
                self.status_pub.publish(String(
                    data=f'导航中 ({i+1}/{len(targets)}): {location}'
                ))

                success = self.navigate_to(x, y)
                if not success:
                    self.get_logger().error(f'导航失败: {location}')
                    self.status_pub.publish(String(data=f'失败: {location}'))
                    break

                self.get_logger().info(f'到达! {location}')
                time.sleep(1.0)
            else:
                self.get_logger().warn(f'未知地点: {location}')

        self.status_pub.publish(String(data='任务完成!'))

    def extract_targets(self, instruction: str) -> list:
        """LLM 从指令中提取目标地点和操作序列"""
        system_prompt = f"""你是一个机器人导航指令解析器。从用户指令中提取目标地点序列。

已知地点:
{json.dumps({k: v for k, v in self.locations.items() if not k.isascii() or k[0].isupper()}, ensure_ascii=False)}

请返回 JSON 格式:
{{"targets": [{{"location": "地点名 (英文 key)", "action": "navigate/scan/wait"}}]}}

如果地点不在已知列表中, 请选择一个最接近的已知地点。
"""
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": instruction}
                ],
                temperature=0.1,
                max_tokens=500,
                response_format={"type": "json_object"},
            )
            content = response.choices[0].message.content.strip()
            result = json.loads(content)
            return result.get('targets', [])
        except Exception as e:
            self.get_logger().error(f'LLM 提取失败: {e}')
            return []

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """调用 Nav2 导航到指定坐标"""
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 不可用')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'发送目标: ({x:.2f}, {y:.2f})')

        future = self.nav_client.send_goal_async(goal)

        # 等待结果 (简化同步等待)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            self.get_logger().error('导航超时')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        if result_future.done():
            result = result_future.result()
            return result.status == 4  # SUCCEEDED
        return False

    def run_test(self):
        """运行测试指令"""
        test_commands = [
            "去 A 货架看看",
            "从入口出发, 经过 B 货架, 最后到办公桌",
            "去充电站转一圈然后到入口",
        ]

        import threading

        def send():
            time.sleep(3)
            for cmd in test_commands:
                self.get_logger().info(f'=== 测试: {cmd} ===')
                targets = self.extract_targets(cmd)
                self.get_logger().info(f'解析结果: {json.dumps(targets, ensure_ascii=False)}')
                time.sleep(1)

        threading.Thread(target=send, daemon=True).start()


def main():
    rclpy.init()
    node = NLNavigation()
    node.run_test()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.2 完整测试流程
```bash
# 终端1: 启动仿真 + Nav2
ros2 launch xbot_sim xbot_full.launch.py world:=office
ros2 launch nav2_bringup navigation_launch.py \
  map:=src/maps/office_map.yaml \
  params_file:=config/nav2_params.yaml \
  use_sim_time:=true

# 终端2: 启动 NL-Navigation
export OPENAI_API_KEY="sk-xxx"
python3 lab14_exercise3.py

# 终端3: 发送自然语言指令
ros2 topic pub /nl/command std_msgs/String \
  "data: '从入口出发, 访问 A 货架和 B 货架, 最后到办公桌'"

# 查看状态
ros2 topic echo /nl/status
```

#### 3.3 完整管道脚本
```python
#!/usr/bin/env python3
"""端到端自然语言导航管道 (一键启动)"""
import subprocess
import time
import threading


def run_full_pipeline(instruction: str):
    """
    完整管道:
    1. 启动 Nav2 (若未启动)
    2. 启动 LLM 规划器
    3. 发送自然语言指令
    4. 监控执行状态
    """
    # 步骤 1 & 2 通过 launch 文件完成
    print('等待 Nav2 就绪...')
    time.sleep(10)

    # 步骤 3: 发布指令
    import rclpy
    from std_msgs.msg import String

    # ... 通过 Python 直接调用 (已有 SimpleCommander 可用)


if __name__ == '__main__':
    instruction = input('请输入自然语言指令: ')
    run_full_pipeline(instruction)
```

### 验收标准
- [ ] LLM 正确解析自然语言指令为目标地点
- [ ] 机器人按序导航到各个目标点
- [ ] 支持中英文混合指令
- [ ] 未知地点能 fallback 到最接近的已知地点
- [ ] Nav2 导航成功到达目标 (误差 < 0.3m)

---

## 练习 4：LLM 指令 → Nav2 导航目标（约 15 分钟）

### 目标
编写节点接收自然语言文本指令，调用 LLM 解析为坐标，通过 Nav2 导航到目标位置。

### 步骤

**步骤1：编写 llm_nav_bridge.py**
```python
#!/usr/bin/env python3
"""练习4: LLM 指令→Nav2 导航目标"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import json
import math


class LLMNavBridge(Node):
    def __init__(self):
        super().__init__('llm_nav_bridge')
        self.nav = BasicNavigator()
        self.cmd_sub = self.create_subscription(
            String, '/nl/command', self.cmd_cb, 10)
        self.locations = {
            'A货架': (2.0, 2.0), 'B货架': (-2.0, 2.0),
            '办公桌': (3.0, -1.0), '充电站': (0.0, -3.0),
            '入口': (0.0, 0.0),
        }

    def cmd_cb(self, msg):
        text = msg.data
        self.get_logger().info(f'收到指令: {text}')
        for loc, (x, y) in self.locations.items():
            if loc in text:
                self.get_logger().info(f'导航到: {loc} ({x:.1f}, {y:.1f})')
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0
                self.nav.waitUntilNav2Active()
                self.nav.goToPose(goal)
                self.get_logger().info(
                    '到达!' if self.nav.isTaskComplete() else '导航中...')
                break


def main():
    rclpy.init()
    node = LLMNavBridge()
    rclpy.spin(node)
    rclpy.shutdown()
```

**步骤2：发送指令测试**
```bash
# 启动 Nav2 后
ros2 topic pub /nl/command std_msgs/String "data: '去 A 货架'"
```

**✓ 验证**：机器人从当前位置导航到 A 货架坐标 (2.0, 2.0)。更换指令为"去充电站"验证切换。

### 思考题
1. 如果指令中包含多个地点（如"先去 A 货架再去 B 货架"），如何实现路径串联？
2. LLM API 不可用时，如何降级为关键词匹配？
