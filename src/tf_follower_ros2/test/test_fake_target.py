import math
import unittest

from tf_follower_ros2.fake_target_broadcaster import compute_target_position


class FakeTargetBroadcasterTest(unittest.TestCase):
    def test_static_target_position(self):
        position = compute_target_position(
            motion_mode="static",
            elapsed_sec=3.0,
            x=3.0,
            y=1.0,
            z=0.0,
            center_x=0.0,
            center_y=0.0,
            radius=0.0,
            angular_speed=0.0,
        )
        self.assertEqual(position, (3.0, 1.0, 0.0))

    def test_circle_target_position(self):
        position = compute_target_position(
            motion_mode="circle",
            elapsed_sec=math.pi / 2.0,
            x=0.0,
            y=0.0,
            z=0.0,
            center_x=3.0,
            center_y=0.0,
            radius=1.0,
            angular_speed=1.0,
        )
        self.assertAlmostEqual(position[0], 3.0, places=6)
        self.assertAlmostEqual(position[1], 1.0, places=6)
        self.assertAlmostEqual(position[2], 0.0, places=6)
