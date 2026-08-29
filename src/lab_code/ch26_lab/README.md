# 第26章 实验代码：车辆纵横向控制

## 文件说明

| 文件 | 说明 |
|------|------|
| `pid_controller.py` | PID控制器实现（带抗积分饱和、微分滤波） |
| `pure_pursuit.py` | Pure Pursuit横向控制器（预瞄距离自适应） |
| `vehicle_ctrl.py` | CARLA车辆控制封装（油门/刹车/转向） |
| `lat_lon_ctrl.py` | 纵横向联合控制主节点 |

## 运行依赖

- Python 3.8+
- CARLA 0.9.16
- NumPy

## 快速开始

```bash
# 单元测试
python pid_controller.py --test

python pure_pursuit.py --test

# CARLA仿真测试
python lat_lon_ctrl.py
```

## 参数调节

参见 `lat_lon_ctrl.py` 中的配置参数。

## 参考

- [PID控制原理](../../../teaching_docs/ch40_车辆纵横向控制.md)

---

## 安装与编译

```bash
pip install numpy
```

## 运行方法

```bash

python pid_controller.py --test     # PID 阶跃响应自检

python pure_pursuit.py --test       # Pure Pursuit 直线跟踪自检

python lat_lon_ctrl.py              # 纵横向联合闭环仿真(30s)

python vehicle_ctrl.py              # CARLA 车辆控制封装演示(需 CARLA)
```

## 验证

`pid_controller.py` 与 `pure_pursuit.py` 提供 `--test` 自检：

```text
$ python pid_controller.py --test
============================================================
PID控制器单元测试: 阶跃响应 (0 -> 10.0)
Kp=2.0, Ki=0.5, Kd=0.1
------------------------------------------------------------
   时间(s)       目标       当前       误差       输出
------------------------------------------------------------
    0.00    10.00     0.05    9.950    1.000
    5.00    10.00     5.05    4.950    1.000
    9.00    10.00     9.05    0.950    1.000
------------------------------------------------------------
✓ 稳态误差达标: 0.1000 (目标 < 0.1)
============================================================
超调量: 0.0%

$ python pure_pursuit.py --test
...
     0     0.25    0.500   -0.006   -0.110     4.50
    45    11.49    0.085   -0.033    0.026     4.50
------------------------------------------------------------
✓ 直线跟踪达标: 横向偏差 0.0538m (目标 < 0.2m)
```

CARLA 闭环测试仍需在仿真器运行时执行 `lat_lon_ctrl.py`。
