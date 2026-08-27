"""slam_map_runner — 委托 slam_sim_demo_ros2 的同名实现.

移动机器人仿真统一使用 `robot_sim_demo`；SLAM 建图 runner 的实现与测试
由 `slam_sim_demo_ros2` 维护，此处仅做转发以便教学时直接运行。
"""
from slam_sim_demo_ros2.slam_map_runner import main

if __name__ == "__main__":
    main()
