import math
import unittest

from tf_follower_ros2.controller import compute_follow_command


class TfFollowerControllerTest(unittest.TestCase):
    def test_stops_inside_distance_threshold(self):
        linear_x, angular_z = compute_follow_command(
            x=0.6,
            y=0.2,
            stop_distance=1.0,
            linear_gain=0.1,
            angular_gain=-0.4,
            max_linear_speed=1.0,
            max_angular_speed=1.5,
        )
        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.0)

    def test_matches_ros1_control_shape_outside_threshold(self):
        linear_x, angular_z = compute_follow_command(
            x=3.0,
            y=4.0,
            stop_distance=1.0,
            linear_gain=0.1,
            angular_gain=-0.4,
            max_linear_speed=1.0,
            max_angular_speed=1.5,
        )
        self.assertAlmostEqual(linear_x, 0.5)
        self.assertAlmostEqual(angular_z, -0.4 * math.atan2(4.0, 3.0))

    def test_clamps_speed_limits(self):
        linear_x, angular_z = compute_follow_command(
            x=10.0,
            y=-10.0,
            stop_distance=1.0,
            linear_gain=0.5,
            angular_gain=-2.0,
            max_linear_speed=1.0,
            max_angular_speed=1.5,
        )
        self.assertEqual(linear_x, 1.0)
        self.assertEqual(angular_z, 1.5)
