import os
import sys


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SRC_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, '..', '..', '..'))
for path in (PACKAGE_DIR, SRC_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

import _ros_stubs  # noqa: F401,E402
