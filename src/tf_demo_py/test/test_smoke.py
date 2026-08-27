import unittest

from tf_demo_py import coordinate_transformation, py_tf_broadcaster, py_tf_listener


class TfDemoSmokeTest(unittest.TestCase):
    def test_tf_demo_modules_import(self):
        self.assertIsNotNone(coordinate_transformation)
        self.assertIsNotNone(py_tf_broadcaster)
        self.assertIsNotNone(py_tf_listener)
