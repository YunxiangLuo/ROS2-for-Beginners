import unittest

from topic_demo_py import pylistener, pytalker


class TopicDemoSmokeTest(unittest.TestCase):
    def test_topic_demo_modules_import(self):
        self.assertIsNotNone(pylistener)
        self.assertIsNotNone(pytalker)
