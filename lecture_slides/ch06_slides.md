# 第6章 PPT：参数系统与 Launch 文件

> 共 14 页

---

## P1 · 标题页
**参数系统与 Launch 文件** | 第6章 | 2课时

## P2 · 学习目标
- 声明和使用参数
- 实现参数动态回调
- YAML 参数文件
- Python Launch 文件编写
- 条件启动与组合启动

## P3 · 参数 CRUD 操作

```python
self.declare_parameter('speed', 1.0)
speed = self.get_parameter('speed').value
self.set_parameters([rclpy.parameter.Parameter('speed', value=2.0)])
```

## P4 · 参数类型支持

| 类型 | 声明示例 |
|------|---------|
| double | `declare_parameter('speed', 1.0)` |
| string | `declare_parameter('name', 'xbot')` |
| bool | `declare_parameter('debug', False)` |
| array | `declare_parameter('ids', [1,2,3])` |

## P5 · 参数回调验证

程序 6-1：
`add_on_set_parameters_callback(fn)` → `SetParametersResult`

## P6 · YAML 参数文件

```yaml
node:
  ros__parameters:
    speed: 2.0
    name: 'xbot'
```

## P7 · 命令行参数操作
```bash
ros2 param list
ros2 param get /node speed
ros2 param set /node speed 3.0
```

## P8 · Launch 文件最小结构

程序 6-2：
```python
def generate_launch_description():
    return LaunchDescription([
        Node(package='pkg', executable='node')
    ])
```

## P9 · 声明与传递参数

```python
use_rviz = LaunchConfiguration('use_rviz', default='false')
DeclareLaunchArgument('use_rviz', ...)
Node(..., parameters=[{'speed': robot_speed}])
```

## P10 · 条件启动

```python
from launch.conditions import IfCondition
Node(..., condition=IfCondition(use_rviz))
```

## P11 · IncludeLaunchDescription

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([pkg_path, '/launch/bringup.py'])
)
```

## P12 · 本章要点
1. 参数 = 键值对，支持声明/获取/设置/回调验证
2. YAML 文件管理批量参数
3. Launch 中 `Node()` 启动节点，`LaunchConfiguration` 传参
4. `IfCondition` 条件启动，`IncludeLaunchDescription` 组合

## P13 · 练习题
1. 参数声明与动态读取
2. 参数回调验证
3. YAML 参数文件加载
4. Python Launch 启动多节点
5. 条件启动 RViz
6. `ros2 param` CLI

## P14 · 第一部分总结
**基础入门 6 章完成！** 下章开始：TF2 坐标变换
