from math import sqrt
import unittest

import cv2
import cv2.aruco as aruco
import numpy as np

from course_lab_utils.vision import (
    ArucoDetectorCompat,
    detect_color_regions,
    rotation_matrix_to_quaternion,
)


class VisionHelpersTest(unittest.TestCase):
    def test_detects_synthetic_green_region(self):
        image = np.zeros((200, 240, 3), dtype=np.uint8)
        cv2.rectangle(image, (60, 50), (180, 150), (0, 255, 0), -1)

        annotated, mask, detections = detect_color_regions(image)

        self.assertEqual(annotated.shape, image.shape)
        self.assertGreater(int(mask.sum()), 0)
        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0]["center"][0], 120, delta=2)
        self.assertAlmostEqual(detections[0]["center"][1], 100, delta=2)

    def test_detects_synthetic_aruco_marker_on_opencv_46(self):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        marker = np.zeros((200, 200), dtype=np.uint8)
        aruco.drawMarker(dictionary, 23, 200, marker, 1)
        canvas = np.full((400, 400), 255, dtype=np.uint8)
        canvas[100:300, 100:300] = marker

        _, ids, _ = ArucoDetectorCompat().detect_markers(canvas)

        self.assertIsNotNone(ids)
        self.assertIn(23, ids.flatten().tolist())

    def test_rotation_matrix_to_quaternion(self):
        matrix = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        quaternion = rotation_matrix_to_quaternion(matrix)
        self.assertAlmostEqual(quaternion[2], sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], sqrt(0.5))
