import unittest

from param_demo_py import param_demo


class ParamDemoSmokeTest(unittest.TestCase):
    def test_param_demo_module_import(self):
        self.assertIsNotNone(param_demo)
