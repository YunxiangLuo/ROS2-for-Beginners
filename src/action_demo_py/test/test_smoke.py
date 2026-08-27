import unittest

from action_demo_py import dishes_client, dishes_server


class ActionDemoSmokeTest(unittest.TestCase):
    def test_action_demo_modules_import(self):
        self.assertIsNotNone(dishes_client)
        self.assertIsNotNone(dishes_server)
