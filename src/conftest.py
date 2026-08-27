"""Expose nested ROS Python package roots to the repository test suite."""

import os
import sys


SRC_DIR = os.path.abspath(os.path.dirname(__file__))
for root, _dirs, files in os.walk(SRC_DIR):
    if 'setup.py' in files and root not in sys.path:
        sys.path.insert(0, root)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import _ros_stubs  # noqa: F401,E402

try:
    import cv2

    if hasattr(cv2, 'aruco') and not hasattr(cv2.aruco, 'drawMarker') \
            and hasattr(cv2.aruco, 'generateImageMarker'):
        cv2.aruco.drawMarker = cv2.aruco.generateImageMarker
except ImportError:
    pass
