"""Chapter 20 vision-LLM caption providers.

Providers are selected with the ``provider`` ROS parameter:

- ``mock``   : offline canned captions, no network access required.
- ``openai`` : any OpenAI-compatible chat/vision endpoint.

Real providers read credentials from environment variables only
(never hard-code them in the source tree or documents):

- ``VISION_API_KEY``  : API token
- ``VISION_BASE_URL`` : e.g. https://<host>/v1
- ``VISION_MODEL``    : model name, e.g. qwen3.5
"""

import base64
import json
import mimetypes
import os
import urllib.request

MOCK_CAPTIONS = (
    'MOCK-VISION: 前方车道畅通，无障碍物，建议保持当前车速巡航。',
    'MOCK-VISION: 检测到前方 8 m 处静止车辆，建议减速并保持安全车距。',
    'MOCK-VISION: 右侧人行道出现行人，注意减速避让。',
)

DEFAULT_PROMPT = '请简要描述摄像头画面中的场景，并给出一条驾驶建议。'


class VisionProvider:
    """Return one caption per call for the configured provider."""

    def __init__(self, provider='mock', prompt='', image_path=''):
        self.provider = provider if provider in ('mock', 'openai') else 'mock'
        self.prompt = prompt or DEFAULT_PROMPT
        self.image_path = image_path
        self._mock_index = 0

    def caption(self):
        if self.provider == 'openai':
            return self._caption_openai()
        return self._caption_mock()

    def _caption_mock(self):
        text = MOCK_CAPTIONS[self._mock_index % len(MOCK_CAPTIONS)]
        self._mock_index += 1
        return text

    def _caption_openai(self):
        api_key = os.environ.get('VISION_API_KEY', '')
        base_url = os.environ.get('VISION_BASE_URL', '')
        model = os.environ.get('VISION_MODEL', '')
        if not api_key or not base_url:
            return ('openai 提供商未配置：请先导出 VISION_API_KEY 与 '
                    'VISION_BASE_URL 环境变量（离线演示可回退 provider:=mock）。')
        content = [{'type': 'text', 'text': self.prompt}]
        image_data = self._encode_image(self.image_path)
        if image_data is not None:
            content.append(image_data)
        payload = {
            'model': model or 'qwen3.5',
            'messages': [{'role': 'user', 'content': content}],
            'max_tokens': 256,
        }
        request = urllib.request.Request(
            base_url.rstrip('/') + '/chat/completions',
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Content-Type': 'application/json',
                'Authorization': 'Bearer ' + api_key,
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=25) as response:
                body = json.loads(response.read().decode('utf-8'))
        except Exception as exc:
            return 'openai 提供商调用失败：{}'.format(exc)
        try:
            text = body['choices'][0]['message']['content'].strip()
        except (KeyError, IndexError, TypeError):
            return 'openai 提供商返回异常：{}'.format(body)
        return text or 'openai 提供商返回空描述。'

    @staticmethod
    def _encode_image(image_path):
        if not image_path:
            return None
        try:
            with open(image_path, 'rb') as image_file:
                raw = image_file.read()
        except OSError:
            return None
        mime = mimetypes.guess_type(image_path)[0] or 'image/png'
        data_url = 'data:{};base64,{}'.format(
            mime, base64.b64encode(raw).decode('ascii'))
        return {'type': 'image_url', 'image_url': {'url': data_url}}
