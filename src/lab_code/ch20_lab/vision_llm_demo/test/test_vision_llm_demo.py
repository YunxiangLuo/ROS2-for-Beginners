import unittest

from vision_llm_demo.vision_provider import VisionProvider


class TestVisionProvider(unittest.TestCase):

    def test_mock_caption_cycles(self):
        provider = VisionProvider(provider='mock')
        first = provider.caption()
        second = provider.caption()
        self.assertTrue(first.startswith('MOCK-VISION:'))
        self.assertNotEqual(first, second)

    def test_unknown_provider_falls_back_to_mock(self):
        provider = VisionProvider(provider='nope')
        self.assertTrue(provider.caption().startswith('MOCK-VISION:'))


if __name__ == '__main__':
    unittest.main()
