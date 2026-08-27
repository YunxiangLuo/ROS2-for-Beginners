import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped, make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    goals = [
        (3.0, 0.0, 0.0),
        (3.0, 2.0, math.radians(90)),
        (0.0, 2.0, math.radians(180)),
        (0.0, 0.0, math.radians(-90)),
    ]

    for i, (x, y, yaw) in enumerate(goals):
        nav.get_logger().info(f'--- Goal {i+1}/{len(goals)} ---')
        goal = make_pose_stamped('map', x, y, yaw)
        nav.goToPose(goal)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                nav.get_logger().info(
                    f'  distance_remaining={feedback.distance_remaining:.2f}m, '
                    f'estimated_time={feedback.estimated_time_remaining.sec}s'
                )

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Goal {i+1} reached!')
        elif result == TaskResult.CANCELED:
            nav.get_logger().warn(f'Goal {i+1} canceled')
        elif result == TaskResult.FAILED:
            nav.get_logger().error(f'Goal {i+1} failed')
        else:
            nav.get_logger().error(f'Goal {i+1} unknown result')

    nav.get_logger().info('All goals completed!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
