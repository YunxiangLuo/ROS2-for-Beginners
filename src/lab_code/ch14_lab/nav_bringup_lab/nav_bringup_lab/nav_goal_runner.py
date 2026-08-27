"""nav_goal_runner — 委托 navigation_sim_demo_ros2 的同名实现.

移动机器人仿真统一使用 `robot_sim_demo`；导航目标发送与监控的实现与测试
由 `navigation_sim_demo_ros2` 维护，此处仅做转发以便教学时直接运行。
"""
from navigation_sim_demo_ros2.nav_goal_runner import main

if __name__ == "__main__":
    main()
