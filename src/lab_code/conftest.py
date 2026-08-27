"""Make nested course packages and ROS import stubs available to lab tests."""

import os
import sys


LAB_CODE_DIR = os.path.abspath(os.path.dirname(__file__))
SRC_DIR = os.path.dirname(LAB_CODE_DIR)
for root, _dirs, files in os.walk(LAB_CODE_DIR):
    if 'setup.py' in files and root not in sys.path:
        sys.path.insert(0, root)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import _ros_stubs  # noqa: F401,E402
