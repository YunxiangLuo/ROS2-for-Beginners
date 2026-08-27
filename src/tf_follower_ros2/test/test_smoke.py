import unittest

from tf_follower_ros2 import controller, fake_target_broadcaster, tf_follower


class TfFollowerSmokeTest(unittest.TestCase):
    def test_tf_follower_modules_import(self):
        self.assertIsNotNone(controller)
        self.assertIsNotNone(fake_target_broadcaster)
        self.assertIsNotNone(tf_follower)
