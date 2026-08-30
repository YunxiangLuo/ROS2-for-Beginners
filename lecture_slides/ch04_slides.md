# 第4章 PPT：服务通信（Services）

> 共 15 页，标注页码 · 图号与教学文档对应

---

## P1 · 标题页

**服务通信（Services）**

- 课程：ROS2 Python 编程
- 章节：第 4 章
- 课时：2 课时

---

## P2 · 本课学习目标

- 理解同步请求-响应通信模型（Client / Server）
- 掌握 .srv 接口定义及 -- 分隔规则
- 掌握 create_service / create_client API
- 区分同步调用与异步调用（Future）
- 掌握超时处理与重试机制
- 熟练使用 ros2 service 命令行工具

---

## P3 · 请求-响应模型

```
Client                                   Server
  │                                         │
  │ ──── Request (call_id, args) ────────►  │
  │                                         │ process_request()
  │ ◄──── Response (call_id, result) ────   │
  │                                         │
```

图 4-1：服务通信请求-响应时序图

- 同步、一问一答，适合「查询状态、触发一次性动作」的短任务
- 一个 Server 可并发处理多个 Client 请求
- 与长时程任务（第 5 章 Action）互补

---

## P4 · .srv 接口定义

```python
# AddTwoInts.srv — 两整数相加服务
int64 a                  # 请求：第一个加数
int64 b                  # 请求：第二个加数
---                      # 分隔线（上：请求，下：响应）
int64 sum                # 响应：相加结果
string message           # 响应：附加消息
```

- 分隔线 `---` 上方定义 Request 字段，下方定义 Response 字段
- 可跨包引用其他接口包的类型（如 `sensor_msgs/Image`）

---

## P5 · Python Server API

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,                 # 服务类型
            'add_two_ints',             # 服务名称
            self.handle_add_two_ints)   # 回调函数

    def handle_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        response.message = \
            f'{request.a} + {request.b} = {response.sum}'
        return response                 # 必须返回 response 对象
```

程序 4-1：Service Server 完整示例——回调签名 `fn(request, response)`

---

## P6 · Python Client API

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(
            AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        # 等待服务上线（标准姿势：先等待、再调用）
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务上线...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # 异步发送请求，返回 future 对象
        future = self.client.call_async(request)
        return future
```

程序 4-2：Service Client 异步调用示例

- 服务端未启动时调用会立即报错，必须先 wait_for_service

---

## P7 · 同步调用

```python
# 同步调用 — 阻塞主线程直到收到响应
req = AddTwoInts.Request()
req.a = 10; req.b = 20
future = self.client.call_async(req)   # 发送异步请求
rclpy.spin_until_future_complete(      # 同步等待完成
    self, future, timeout_sec=5.0)
result = future.result()               # 获取结果
self.get_logger().info(f'Result: {result.sum}')
```

- `spin_until_future_complete` 阻塞直到 Future 完成或超时
- 服务回调运行在 spin 的执行线程内，复杂工作应移出，避免阻塞其他回调

---

## P8 · 异步调用

```python
# 异步调用 — 通过回调处理结果，不阻塞
def send_request_async(self, a, b):
    req = AddTwoInts.Request()
    req.a = a; req.b = b
    future = self.client.call_async(req)
    future.add_done_callback(self.response_callback)

def response_callback(self, future):
    try:
        result = future.result()
        self.get_logger().info(f'Result: {result.sum}')
    except Exception as e:
        self.get_logger().error(f'调用失败: {e}')
```

- `call_async` 返回 Future，可注册完成回调
- 注意在回调中捕获异常

---

## P9 · 超时处理

```python
def call_with_timeout(self, a, b, timeout=5.0):
    req = AddTwoInts.Request()
    req.a = a; req.b = b
    future = self.client.call_async(req)
    start_time = time.time()

    # 带超时的轮询
    while rclpy.ok():
        rclpy.spin_once(self, timeout_sec=0.1)
        if future.done():
            return future.result()
        if time.time() - start_time > timeout:
            self.get_logger().error('服务调用超时！')
            future.cancel()
            return None
```

程序 4-3：服务调用超时处理模式

---

## P10 · 重试机制

```python
def call_with_retry(self, a, b, max_retries=3):
    for attempt in range(max_retries):
        if self.client.wait_for_service(timeout_sec=2.0):
            req = AddTwoInts.Request()
            req.a = a; req.b = b
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=5.0)
            if future.result() is not None:
                return future.result()
        self.get_logger().warn(
            f'重试 {attempt + 1}/{max_retries}...')
    self.get_logger().error('所有重试均失败！')
    return None
```

- 每次重试前 wait_for_service 检测服务端是否上线

---

## P11 · 三层容错设计

```
        ┌──────────────────────────────┐
        │ 第 1 层：超时                 │  spin_until_future_complete
        │     超过时限即放弃            │  (timeout_sec)
        ├──────────────────────────────┤
        │ 第 2 层：重试                 │  有限次退避重试
        │     超时/网络错误就重试       │  call_with_retry
        ├──────────────────────────────┤
        │ 第 3 层：降级                 │  切换备用策略
        │     多次失败用本地默认值      │  （如本地默认值）
        └──────────────────────────────┘
```

- 服务端可能未启动、网络可能抖动，是真实机器人系统的基本假设
- The Construct 称之为「健壮服务客户端模式 (Robust Service Client Pattern)」

---

## P12 · 服务命令行工具

```bash
ros2 service list            # 列出所有服务
ros2 service type /spawn     # 查询服务类型
ros2 service find            # 按类型查找服务
ros2 service call /spawn turtlesim/srv/Spawn \
    "{x: 2.0, y: 2.0, theta: 0.0, name: ''}"
```

- 命令行直接调用：不写一行代码即可验证服务端逻辑
- 小乌龟示例服务：/spawn、/clear、/set_pen

---

## P13 · 本章要点

1. 服务通信 = 同步请求-响应，Client 发 Request，Server 回 Response
2. .srv 文件：`---` 上方 Request、下方 Response
3. Server：create_service(类型, 名称, 回调)，回调须返回 response
4. Client：create_client + wait_for_service + call_async
5. 同步调用 spin_until_future_complete；异步调用 add_done_callback
6. 生产环境必须处理超时 + 重试 + 降级三层容错

---

## P14 · 练习题

1. 基于 AddTwoInts 编写 Server 和 Client 节点，验证加法功能
2. 设计 WeatherQuery.srv（输入：城市 string；输出：温度 float64 + 天气 string），实现查询服务
3. 测试服务超时：Client 设 1 秒超时，Server 处理 3 秒，观察超时行为
4. 编写带重试的 Client（client_wait.py）：max_retries=3；先只启动 `ros2 run service_demo client_wait 5 10` 观察重试日志，再启动 server
5. 多个 Client 同时请求同一个 Server，观察并发处理行为
6. 使用 ros2 service call 命令行调用服务，验证 Server 响应

> 提示：练习 4 需 `colcon build --packages-select service_demo --symlink-install`；重试间 sleep 2 秒

---

## P15 · 下章预告

**第 5 章：动作通信（Actions）**

- 目标 (goal) / 反馈 (feedback) / 结果 (result) 三部分模型
- 客户端发起、可取消、长时程任务
- 动作服务器 / 动作客户端编程实战