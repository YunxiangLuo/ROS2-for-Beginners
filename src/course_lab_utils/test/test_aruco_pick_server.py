import unittest

from geometry_msgs.msg import PoseStamped

from course_lab_utils.aruco_pick_server import make_grasps, make_places


class ArucoPickMessagesTest(unittest.TestCase):
    def test_builds_grasp_candidates(self):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.1
        pose.pose.orientation.w = 1.0

        grasps = make_grasps(pose, "aruco_7", "table")

        self.assertEqual(len(grasps), 5)
        self.assertIn("aruco_7", grasps[0].allowed_touch_objects)
        self.assertEqual(len(grasps[0].pre_grasp_posture.points), 1)

    def test_builds_place_candidates_without_mutating_input(self):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.25
        pose.pose.position.y = 0.25
        pose.pose.orientation.w = 1.0

        places = make_places(pose)

        self.assertEqual(len(places), 9)
        self.assertEqual(pose.pose.position.x, 0.25)
        self.assertEqual(pose.pose.position.y, 0.25)
