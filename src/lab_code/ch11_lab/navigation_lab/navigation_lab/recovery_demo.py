import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped, make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    nav.get_logger().info('Navigating toward obstacles to trigger recovery...')
    goal = make_pose_stamped('map', 4.0, 0.0, 0.0)
    nav.goToPose(goal)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            nav.get_logger().info(
                f'distance_remaining={feedback.distance_remaining:.2f}m, '
                f'navigation_time={feedback.navigation_time.sec}s, '
                f'estimated_time_remaining={feedback.estimated_time_remaining.sec}s'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Goal reached!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Canceled')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('Failed — recovery behaviors were triggered')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
