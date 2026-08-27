# -*- coding: utf-8 -*-
"""av_carla_interfaces 消息定义一致性测试。

验证：
1. CMakeLists.txt 中注册的 msg/srv/action 文件均存在且非空；
2. .msg 文件字段定义语法合法；
3. av_control_cpp 源码中引用的消息字段在 .msg 定义中确实存在
   （防止 C++ 编译期/运行期错误，本机无编译器故做静态校验）；
4. av_perception_py / av_safety_monitor 中引用的自定义消息已注册。
"""

import os
import re

import pytest

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
INTERFACES_DIR = os.path.join(SRC_DIR, 'av_carla_interfaces')


def _parse_msg_fields(msg_path):
    """解析 .msg 文件，返回字段名列表。"""
    fields = []
    with open(msg_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue
            parts = line.split()
            # 形如 "float64 throttle" 或 "Cluster[] clusters" 或 "std_msgs/Header header"
            m = re.match(r'^([A-Za-z0-9_/]+)(\[\])?\s+([A-Za-z0-9_]+)$', line)
            assert m, 'invalid msg line: %r in %s' % (line, msg_path)
            fields.append(m.group(3))
    return fields


def _cmake_lists_entries(cmake_path):
    """提取 CMakeLists.txt 中 msg_files/srv_files/action_files 的条目。"""
    with open(cmake_path, 'r', encoding='utf-8') as f:
        content = f.read()
    entries = {}
    for name in ('msg_files', 'srv_files', 'action_files'):
        m = re.search(r'set\(%s\s+(.*?)\)' % name, content, re.S)
        if m:
            entries[name] = re.findall(r'"([^"]+)"', m.group(1))
    return entries


def test_cmake_registered_files_exist():
    entries = _cmake_lists_entries(os.path.join(INTERFACES_DIR, 'CMakeLists.txt'))
    assert entries, 'CMakeLists.txt must register interface files'
    for group, files in entries.items():
        assert files, '%s should not be empty' % group
        for rel in files:
            path = os.path.join(INTERFACES_DIR, rel)
            assert os.path.isfile(path), 'missing file registered in CMakeLists: %s' % rel
            assert os.path.getsize(path) > 0, 'empty interface file: %s' % rel


def test_msg_field_syntax():
    msg_dir = os.path.join(INTERFACES_DIR, 'msg')
    msg_files = [f for f in os.listdir(msg_dir) if f.endswith('.msg')]
    assert len(msg_files) >= 9
    for fn in msg_files:
        fields = _parse_msg_fields(os.path.join(msg_dir, fn))
        assert len(fields) > 0, '%s has no fields' % fn


def test_cpp_referenced_fields_exist():
    """av_control_cpp 回调函数中 msg->field 访问的字段必须在 .msg 中定义。"""
    msg_fields = {}
    for fn in os.listdir(os.path.join(INTERFACES_DIR, 'msg')):
        name = fn[:-4]
        msg_fields[name] = set(_parse_msg_fields(os.path.join(INTERFACES_DIR, 'msg', fn)))

    cpp_dir = os.path.join(SRC_DIR, 'av_control_cpp', 'src')
    errors = []
    func_pattern = re.compile(
        r'void\s+(\w+)\s*\(\s*const\s+av_carla_interfaces::msg::(\w+)::SharedPtr\s+(\w+)\s*\)\s*\{(.*?)\n  \}',
        re.S)
    for fn in sorted(os.listdir(cpp_dir)):
        if not fn.endswith('.cpp'):
            continue
        with open(os.path.join(cpp_dir, fn), 'r', encoding='utf-8') as f:
            src = f.read()
        for m in func_pattern.finditer(src):
            func_name, msg_name, var_name, body = m.groups()
            if msg_name not in msg_fields:
                errors.append('%s: %s() uses unknown msg %s' % (fn, func_name, msg_name))
                continue
            for access in re.findall(re.escape(var_name) + r'->([A-Za-z_][A-Za-z0-9_]*)', body):
                if access not in msg_fields[msg_name]:
                    errors.append('%s: %s() msg %s has no field %r'
                                  % (fn, func_name, msg_name, access))
    assert not errors, '\n'.join(errors)

def test_python_referenced_messages_registered():
    """Python 包中 import 的 av_carla_interfaces 消息必须已在 CMakeLists 注册。"""
    entries = _cmake_lists_entries(os.path.join(INTERFACES_DIR, 'CMakeLists.txt'))
    registered = set()
    for group in ('msg_files', 'srv_files', 'action_files'):
        for rel in entries.get(group, []):
            registered.add(os.path.splitext(os.path.basename(rel))[0])

    import_patterns = [
        (os.path.join(SRC_DIR, 'av_perception_py'), ('av_perception_py',)),
        (os.path.join(SRC_DIR, 'av_planning_py'), ('av_planning_py',)),
        (os.path.join(SRC_DIR, 'av_safety_monitor'), ('av_safety_monitor',)),
    ]
    errors = []
    for pkg_root, _ in import_patterns:
        for root, _dirs, files in os.walk(os.path.join(pkg_root, pkg_root.split(os.sep)[-1])):
            for fn in files:
                if not fn.endswith('.py'):
                    continue
                with open(os.path.join(root, fn), 'r', encoding='utf-8') as f:
                    src = f.read()
                for name in re.findall(
                        r'from av_carla_interfaces\.(?:msg|srv|action) import ([A-Za-z0-9_, ]+)',
                        src):
                    for token in re.split(r'[,\s]+', name.strip()):
                        if token:
                            if token not in registered:
                                errors.append('%s imports unregistered %s' % (fn, token))
    assert not errors, '\n'.join(errors)


def test_waypoint_fields_match_cpp_usage():
    """Waypoint.msg 字段必须与 C++ 中 wp.x / wp.y 平面字段用法一致。"""
    fields = set(_parse_msg_fields(os.path.join(INTERFACES_DIR, 'msg', 'Waypoint.msg')))
    assert {'x', 'y', 'z', 'speed'} <= fields
    assert 'position' not in fields, 'C++ uses flat x/y fields, not position'


def test_ego_state_fields_match_cpp_usage():
    fields = set(_parse_msg_fields(os.path.join(INTERFACES_DIR, 'msg', 'EgoState.msg')))
    assert 'speed' in fields
    assert 'pose' in fields
    assert 'twist' in fields
    # C++ 曾经引用过不存在字段，这里锁定真实字段集，防止回归
    assert 'target_speed' not in fields
    assert 'velocity' not in fields




