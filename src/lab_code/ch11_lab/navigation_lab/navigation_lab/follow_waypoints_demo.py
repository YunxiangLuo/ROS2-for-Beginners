import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped, make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    waypoints = [
        make_pose_stamped('map', 2.0, 0.0, 0.0),
        make_pose_stamped('map', 2.0, 2.0, math.radians(90)),
        make_pose_stamped('map', 0.0, 2.0, math.radians(180)),
        make_pose_stamped('map', 0.0, 0.0, math.radians(-90)),
    ]

    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            nav.get_logger().info(
                f'current_waypoint={feedback.current_waypoint}/{len(waypoints)}'
            )

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('All waypoints reached!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Waypoint task canceled')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('Waypoint task failed')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
