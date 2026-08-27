import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import make_pose_stamped, make_pose_stamped


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose_stamped('map', 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    patrol_points = [
        make_pose_stamped('map', 2.0, 0.0, 0.0),
        make_pose_stamped('map', 2.0, 2.0, math.radians(90)),
        make_pose_stamped('map', 0.0, 2.0, math.radians(180)),
        make_pose_stamped('map', 0.0, 0.0, math.radians(-90)),
    ]

    loops = 2

    for lap in range(loops):
        nav.get_logger().info(f'=== Patrol lap {lap+1}/{loops} ===')

        nav.followWaypoints(patrol_points)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                nav.get_logger().info(
                    f'current_waypoint={feedback.current_waypoint}/{len(patrol_points)}'
                )

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Lap {lap+1} complete!')
        else:
            nav.get_logger().error(f'Lap {lap+1} interrupted')
            break

    nav.get_logger().info('Patrol finished!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
