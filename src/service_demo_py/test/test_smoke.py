import unittest

from service_demo_py import client_demo, server_demo


class ServiceDemoSmokeTest(unittest.TestCase):
    def test_service_demo_modules_import(self):
        self.assertIsNotNone(client_demo)
        self.assertIsNotNone(server_demo)
