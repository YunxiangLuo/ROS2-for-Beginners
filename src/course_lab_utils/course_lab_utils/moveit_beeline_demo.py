"""Cartesian and point-to-point triangle planning example."""

from copy import deepcopy
import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node

from .moveit2 import (
    compute_cartesian_path,
    get_current_pose,
    plan_and_execute,
    set_named_goal,
    set_pose_goal,
)


class MoveItBeelineDemo(Node):
    ARM_GROUP = "xarm"
    END_EFFECTOR_LINK = "gripper_centor_link"
    REFERENCE_FRAME = "base_link"

    def __init__(self):
        super().__init__("moveit_beeline_demo")
        self.declare_parameter("cartesian", True)
        self.moveit = MoveItPy(node_name="moveit_beeline_demo_moveit")
        self.arm = self.moveit.get_planning_component(self.ARM_GROUP)

    def run(self):
        set_named_goal(self.arm, "Home")
        self._execute("Home")

        target = PoseStamped()
        target.header.frame_id = self.REFERENCE_FRAME
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = 0.4
        target.pose.position.z = 0.45
        target.pose.orientation.w = 1.0
        set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
        self._execute("triangle start")

        start = get_current_pose(self.moveit, self.END_EFFECTOR_LINK)
        waypoint = deepcopy(start)
        waypoints = [deepcopy(waypoint)]
        waypoint.position.z -= 0.2
        waypoint.position.y -= 0.1
        waypoints.append(deepcopy(waypoint))
        waypoint.position.y += 0.2
        waypoints.append(deepcopy(waypoint))
        waypoints.append(deepcopy(start))

        if bool(self.get_parameter("cartesian").value):
            fraction, trajectory = compute_cartesian_path(
                self,
                self.moveit,
                self.ARM_GROUP,
                self.END_EFFECTOR_LINK,
                self.REFERENCE_FRAME,
                waypoints,
            )
            if trajectory is None or fraction < 0.999:
                raise RuntimeError(f"Cartesian planning incomplete: {fraction:.1%}")
            self.moveit.execute(trajectory, controllers=[])
        else:
            for index, pose in enumerate(waypoints):
                target.header.stamp = self.get_clock().now().to_msg()
                target.pose = pose
                set_pose_goal(self.arm, target, self.END_EFFECTOR_LINK)
                self._execute(f"waypoint {index}")

        set_named_goal(self.arm, "Home")
        self._execute("Home")

    def _execute(self, description: str):
        if not plan_and_execute(self.moveit, self.arm):
            raise RuntimeError(f"Planning failed for {description}")
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItBeelineDemo()
    try:
        node.run()
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
