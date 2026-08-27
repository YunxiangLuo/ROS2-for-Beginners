"""Minimal ROS 2 import stubs for algorithm tests without a ROS install.

The package tests load this module from their local conftest files. Existing
ROS modules are left untouched, so a sourced ROS 2 environment still uses the
real message and client libraries.
"""

from __future__ import annotations

import importlib
import sys
import time
from types import ModuleType, SimpleNamespace


_MESSAGE_SUBSCRIBERS = {}
_TRANSFORMS = {}


def _try_import(name):
    try:
        return importlib.import_module(name)
    except ImportError:
        return None


def _module(name):
    existing = _try_import(name)
    if existing is not None:
        return existing

    module = ModuleType(name)
    sys.modules[name] = module
    if "." in name:
        parent_name, child_name = name.rsplit(".", 1)
        parent = _module(parent_name)
        setattr(parent, child_name, module)
    return module


class _Message:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class _Time:
    def __init__(self, nanoseconds=None):
        self.nanoseconds = int(time.time() * 1e9 if nanoseconds is None else nanoseconds)

    def to_msg(self):
        return self


class _Clock:
    def now(self):
        return _Time()


class _Logger:
    def debug(self, *args, **kwargs):
        pass

    info = debug
    warn = debug
    warning = debug
    error = debug
    fatal = debug


class _Publisher:
    def __init__(self, topic):
        self.topic_name = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)
        for callback in list(_MESSAGE_SUBSCRIBERS.get(self.topic_name, [])):
            callback(message)

    def get_subscription_count(self):
        return 0


class _Subscription:
    def __init__(self, topic, callback):
        self.topic_name = topic
        self.callback = callback


class _Timer:
    def __init__(self, period, callback):
        self.period = max(float(period), 0.001)
        self.callback = callback
        self.next_call = time.monotonic() + self.period
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Node:
    def __init__(self, name="node", **kwargs):
        self.name = name
        self._parameters = {}
        self.publishers = []
        self.subscriptions = []
        self.services = []
        self.timers = []
        self._logger = _Logger()
        self._clock = _Clock()
        self._parameter_callbacks = []

    def declare_parameter(self, name, value=None):
        self._parameters[name] = value
        return SimpleNamespace(value=value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self._parameters.get(name))

    def set_parameters(self, parameters):
        results = [callback(parameters) for callback in self._parameter_callbacks]
        if not results or all(getattr(result, 'successful', True) for result in results):
            for parameter in parameters:
                self._parameters[parameter.name] = parameter.value
        return results

    def add_on_set_parameters_callback(self, callback):
        self._parameter_callbacks.append(callback)

    def create_publisher(self, message_type, topic, qos, **kwargs):
        publisher = _Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        subscription = _Subscription(topic, callback)
        self.subscriptions.append(subscription)
        _MESSAGE_SUBSCRIBERS.setdefault(topic, []).append(callback)
        return subscription

    def create_service(self, service_type, name, callback, **kwargs):
        service = SimpleNamespace(service_name=name, callback=callback)
        self.services.append(service)
        return service

    def create_client(self, service_type, name, **kwargs):
        return SimpleNamespace(
            service_name=name,
            wait_for_service=lambda timeout_sec=None: False,
            call_async=lambda request: SimpleNamespace(result=lambda: None),
        )

    def create_timer(self, period, callback, **kwargs):
        timer = _Timer(period, callback)
        self.timers.append(timer)
        return timer

    def create_lifecycle_publisher(self, message_type, topic, qos, **kwargs):
        return self.create_publisher(message_type, topic, qos, **kwargs)

    def destroy_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def destroy_publisher(self, publisher):
        if publisher in self.publishers:
            self.publishers.remove(publisher)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


class _CallbackGroup:
    pass


class _Executor:
    def __init__(self, *args, **kwargs):
        self.nodes = []
        self._shutdown = False

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes.append(node)

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)

    def spin(self):
        while not self._shutdown:
            now = time.monotonic()
            for node in list(self.nodes):
                for timer in list(getattr(node, 'timers', [])):
                    if timer.cancelled or now < timer.next_call:
                        continue
                    timer.next_call = now + timer.period
                    timer.callback()
            time.sleep(0.005)

    def shutdown(self):
        self._shutdown = True


def _install_rclpy():
    rclpy = _module("rclpy")
    if not hasattr(rclpy, "init"):
        rclpy.init = lambda args=None: None
        rclpy.shutdown = lambda: None
        rclpy.ok = lambda: True
        rclpy.spin = lambda node: None
        rclpy.spin_once = lambda node, timeout_sec=None: None
        rclpy.spin_until_future_complete = lambda node, future, timeout_sec=None: None

    node = _module("rclpy.node")
    node.Node = _Node

    callbacks = _module("rclpy.callback_groups")
    callbacks.ReentrantCallbackGroup = _CallbackGroup
    callbacks.MutuallyExclusiveCallbackGroup = _CallbackGroup

    executors = _module("rclpy.executors")
    executors.MultiThreadedExecutor = _Executor
    executors.SingleThreadedExecutor = _Executor

    class _ReliabilityPolicy:
        RELIABLE = 1
        BEST_EFFORT = 2

    class _DurabilityPolicy:
        VOLATILE = 1
        TRANSIENT_LOCAL = 2

    class _HistoryPolicy:
        KEEP_LAST = 1
        KEEP_ALL = 2

    class _QoSProfile:
        def __init__(self, depth=10, reliability=None, durability=None, history=None, **kwargs):
            self.depth = depth
            self.reliability = reliability
            self.durability = durability
            self.history = history

    qos = _module("rclpy.qos")
    qos.QoSProfile = _QoSProfile
    qos.ReliabilityPolicy = _ReliabilityPolicy
    qos.DurabilityPolicy = _DurabilityPolicy
    qos.HistoryPolicy = _HistoryPolicy

    parameter = _module("rclpy.parameter")

    class _Parameter:
        class Type:
            BOOL = 1
            INTEGER = 2
            DOUBLE = 3
            STRING = 4

        def __init__(self, name, parameter_type_or_value=None, value=None, **kwargs):
            self.name = name
            self.type = parameter_type_or_value if value is not None else None
            self.value = parameter_type_or_value if value is None else value

    parameter.Parameter = _Parameter

    utilities = _module("rclpy.utilities")
    utilities.remove_ros_args = lambda args: list(args)

    duration = _module("rclpy.duration")

    class _Duration:
        def __init__(self, seconds=0.0, **kwargs):
            self.nanoseconds = int(seconds * 1e9)

        def to_msg(self):
            return self

    duration.Duration = _Duration

    ros_time = _module("rclpy.time")
    ros_time.Time = _Time

    action = _module("rclpy.action")

    class _ActionServer:
        def __init__(self, *args, **kwargs):
            pass

    action.ActionServer = _ActionServer

    class _ActionClient:
        def __init__(self, *args, **kwargs):
            pass

        def wait_for_server(self, timeout_sec=None):
            return False

    action.ActionClient = _ActionClient
    action.GoalResponse = SimpleNamespace(ACCEPT=1, REJECT=2)
    action.CancelResponse = SimpleNamespace(ACCEPT=1, REJECT=2)

    lifecycle = _module("rclpy.lifecycle")

    class _TransitionCallbackReturn:
        SUCCESS = 1
        FAILURE = 2
        ERROR = 3

    class _LifecycleNode(_Node):
        def create_lifecycle_publisher(self, message_type, topic, qos, **kwargs):
            return self.create_publisher(message_type, topic, qos, **kwargs)

        def trigger_configure(self):
            return self.on_configure(SimpleNamespace())

        def trigger_activate(self):
            return self.on_activate(SimpleNamespace())

        def trigger_deactivate(self):
            return self.on_deactivate(SimpleNamespace())

        def trigger_cleanup(self):
            return self.on_cleanup(SimpleNamespace())

        def on_activate(self, state):
            del state
            return _TransitionCallbackReturn.SUCCESS

        def on_deactivate(self, state):
            del state
            return _TransitionCallbackReturn.SUCCESS

    lifecycle.LifecycleNode = _LifecycleNode
    lifecycle.TransitionCallbackReturn = _TransitionCallbackReturn


def _install_messages():
    std = _module("std_msgs.msg")

    class Header(_Message):
        def __init__(self, frame_id="", stamp=None, **kwargs):
            super().__init__(frame_id=frame_id, stamp=stamp or _Time(), **kwargs)

    class String(_Message):
        def __init__(self, data="", **kwargs):
            super().__init__(data=data, **kwargs)

    class Bool(_Message):
        def __init__(self, data=False, **kwargs):
            super().__init__(data=data, **kwargs)

    class Float32(_Message):
        def __init__(self, data=0.0, **kwargs):
            super().__init__(data=data, **kwargs)

    class Float64(Float32):
        pass

    class Int32(_Message):
        def __init__(self, data=0, **kwargs):
            super().__init__(data=data, **kwargs)

    class Int64(Int32):
        pass

    class Float32MultiArray(_Message):
        def __init__(self, data=None, **kwargs):
            super().__init__(data=[] if data is None else data, **kwargs)

    class ColorRGBA(_Message):
        def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0, **kwargs):
            super().__init__(r=r, g=g, b=b, a=a, **kwargs)

    for cls in (Header, String, Bool, Float32, Float64, Int32, Int64,
                Float32MultiArray, ColorRGBA):
        if not hasattr(std, cls.__name__):
            setattr(std, cls.__name__, cls)

    geometry = _module("geometry_msgs.msg")

    class Vector3(_Message):
        def __init__(self, x=0.0, y=0.0, z=0.0, **kwargs):
            super().__init__(x=x, y=y, z=z, **kwargs)

    class Point(Vector3):
        pass

    class Quaternion(_Message):
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0, **kwargs):
            super().__init__(x=x, y=y, z=z, w=w, **kwargs)

    class Pose(_Message):
        def __init__(self, position=None, orientation=None, **kwargs):
            super().__init__(position=position or Point(),
                             orientation=orientation or Quaternion(), **kwargs)

    class PoseStamped(_Message):
        def __init__(self, header=None, pose=None, **kwargs):
            super().__init__(header=header or Header(), pose=pose or Pose(), **kwargs)

    class PoseArray(_Message):
        def __init__(self, header=None, poses=None, **kwargs):
            super().__init__(header=header or Header(), poses=[] if poses is None else poses, **kwargs)

    class PoseWithCovarianceStamped(_Message):
        def __init__(self, header=None, pose=None, **kwargs):
            super().__init__(header=header or Header(),
                             pose=pose or SimpleNamespace(pose=Pose(), covariance=[0.0] * 36),
                             **kwargs)

    class Twist(_Message):
        def __init__(self, linear=None, angular=None, **kwargs):
            super().__init__(linear=linear or Vector3(), angular=angular or Vector3(), **kwargs)

    class TwistStamped(_Message):
        def __init__(self, header=None, twist=None, **kwargs):
            super().__init__(header=header or Header(), twist=twist or Twist(), **kwargs)

    class Pose2D(_Message):
        def __init__(self, x=0.0, y=0.0, theta=0.0, **kwargs):
            super().__init__(x=x, y=y, theta=theta, **kwargs)

    class PointStamped(_Message):
        def __init__(self, header=None, point=None, **kwargs):
            super().__init__(header=header or Header(), point=point or Point(), **kwargs)

    class TransformStamped(_Message):
        def __init__(self, header=None, child_frame_id="", transform=None, **kwargs):
            super().__init__(header=header or Header(), child_frame_id=child_frame_id,
                             transform=transform or SimpleNamespace(
                                 translation=Vector3(), rotation=Quaternion()), **kwargs)

    class CameraInfo(_Message):
        pass

    for cls in (Vector3, Point, Quaternion, Pose, PoseStamped, PoseArray,
                PoseWithCovarianceStamped, Twist, TwistStamped, Pose2D,
                PointStamped, TransformStamped, CameraInfo):
        if not hasattr(geometry, cls.__name__):
            setattr(geometry, cls.__name__, cls)

    sensor = _module("sensor_msgs.msg")

    class Image(_Message):
        def __init__(self, header=None, **kwargs):
            super().__init__(header=header or Header(), **kwargs)

    class PointCloud2(_Message):
        def __init__(self, header=None, fields=None, data=b"", point_step=0, **kwargs):
            super().__init__(header=header or Header(), fields=[] if fields is None else fields,
                             data=data, point_step=point_step, **kwargs)

    class PointField(_Message):
        INT8 = 1
        UINT8 = 2
        INT16 = 3
        UINT16 = 4
        INT32 = 5
        UINT32 = 6
        FLOAT32 = 7
        FLOAT64 = 8

    class NavSatFix(_Message):
        pass

    class Imu(_Message):
        pass

    class CameraInfo(_Message):
        pass

    class LaserScan(_Message):
        def __init__(self, header=None, ranges=None, **kwargs):
            super().__init__(header=header or Header(),
                             ranges=[] if ranges is None else ranges, **kwargs)

    class JointState(_Message):
        def __init__(self, header=None, name=None, position=None, velocity=None, effort=None, **kwargs):
            super().__init__(header=header or Header(),
                             name=[] if name is None else name,
                             position=[] if position is None else position,
                             velocity=[] if velocity is None else velocity,
                             effort=[] if effort is None else effort, **kwargs)

    for cls in (Image, PointCloud2, PointField, NavSatFix, Imu, CameraInfo,
                LaserScan, JointState):
        if not hasattr(sensor, cls.__name__):
            setattr(sensor, cls.__name__, cls)

    nav = _module("nav_msgs.msg")

    class OccupancyGrid(_Message):
        def __init__(self, info=None, data=None, **kwargs):
            super().__init__(info=info, data=[] if data is None else data, **kwargs)

    class Path(_Message):
        def __init__(self, header=None, poses=None, **kwargs):
            super().__init__(header=header or Header(), poses=[] if poses is None else poses, **kwargs)

    class Odometry(_Message):
        def __init__(self, header=None, child_frame_id="", pose=None, twist=None, **kwargs):
            super().__init__(header=header or Header(), child_frame_id=child_frame_id,
                             pose=pose or SimpleNamespace(pose=Pose()),
                             twist=twist or SimpleNamespace(twist=Twist()), **kwargs)

    for cls in (OccupancyGrid, Path, Odometry):
        if not hasattr(nav, cls.__name__):
            setattr(nav, cls.__name__, cls)

    visualization = _module("visualization_msgs.msg")

    class Marker(_Message):
        CUBE = 1
        SPHERE = 2
        ADD = 0

        def __init__(self, header=None, pose=None, scale=None, color=None, lifetime=None, **kwargs):
            super().__init__(header=header or Header(), pose=pose or Pose(),
                             scale=scale or Vector3(1.0, 1.0, 1.0),
                             color=color or ColorRGBA(),
                             lifetime=lifetime or SimpleNamespace(sec=0, nanosec=0), **kwargs)

    class MarkerArray(_Message):
        def __init__(self, markers=None, **kwargs):
            super().__init__(markers=[] if markers is None else markers, **kwargs)

    for cls in (Marker, MarkerArray):
        if not hasattr(visualization, cls.__name__):
            setattr(visualization, cls.__name__, cls)

    vision = _module("vision_msgs.msg")

    class ObjectHypothesis(_Message):
        def __init__(self, class_id="", score=0.0, **kwargs):
            super().__init__(class_id=class_id, score=score, **kwargs)

    class ObjectHypothesisWithPose(_Message):
        def __init__(self, hypothesis=None, pose=None, **kwargs):
            super().__init__(hypothesis=hypothesis or ObjectHypothesis(),
                             pose=pose or Pose(), **kwargs)

    class Detection2D(_Message):
        def __init__(self, bbox=None, results=None, **kwargs):
            super().__init__(bbox=bbox or SimpleNamespace(
                                 center=SimpleNamespace(position=Point(),
                                                         theta=0.0),
                                 size_x=0.0, size_y=0.0),
                             results=[] if results is None else results, **kwargs)

    class Detection2DArray(_Message):
        def __init__(self, header=None, detections=None, **kwargs):
            super().__init__(header=header or Header(),
                             detections=[] if detections is None else detections, **kwargs)

    for cls in (ObjectHypothesis, ObjectHypothesisWithPose, Detection2D, Detection2DArray):
        if not hasattr(vision, cls.__name__):
            setattr(vision, cls.__name__, cls)

    builtin = _module("builtin_interfaces.msg")
    if not hasattr(builtin, "Duration"):
        builtin.Duration = type("Duration", (_Message,), {"__init__": lambda self, sec=0, nanosec=0, **kw: _Message.__init__(self, sec=sec, nanosec=nanosec, **kw)})
    if not hasattr(builtin, "Time"):
        builtin.Time = _Time

    services = _module("std_srvs.srv")

    class _Trigger:
        Request = type("Request", (), {})
        Response = type("Response", (), {"__init__": lambda self, success=False, message="": setattr(self, "success", success) or setattr(self, "message", message)})

    class _SetBool:
        Request = type("Request", (), {"__init__": lambda self, data=False: setattr(self, "data", data)})
        Response = type("Response", (), {"__init__": lambda self, success=False, message="": setattr(self, "success", success) or setattr(self, "message", message)})

    if not hasattr(services, "Trigger"):
        services.Trigger = _Trigger
    if not hasattr(services, "SetBool"):
        services.SetBool = _SetBool

    diagnostic = _module("diagnostic_msgs.msg")
    for name in ("DiagnosticStatus", "DiagnosticArray", "KeyValue"):
        if not hasattr(diagnostic, name):
            setattr(diagnostic, name, type(name, (_Message,), {}))


def _install_course_messages():
    messages = _module("av_carla_interfaces.msg")
    Header = getattr(_module("std_msgs.msg"), "Header")
    Point = getattr(_module("geometry_msgs.msg"), "Point")
    Pose = getattr(_module("geometry_msgs.msg"), "Pose")
    PoseStamped = getattr(_module("geometry_msgs.msg"), "PoseStamped")
    Twist = getattr(_module("geometry_msgs.msg"), "Twist")

    class Cluster(_Message):
        def __init__(self, centroid=None, points=None, **kwargs):
            super().__init__(centroid=centroid or Pose(), points=[] if points is None else points, **kwargs)

    class ClusterArray(_Message):
        def __init__(self, header=None, clusters=None, **kwargs):
            super().__init__(header=header or Header(), clusters=[] if clusters is None else clusters, **kwargs)

    class PerceptionObject(_Message):
        def __init__(self, pose=None, velocity=None, id="", confidence=0.0, **kwargs):
            super().__init__(pose=pose or Pose(), velocity=velocity or Point(), id=id,
                             confidence=confidence, **kwargs)

    class PerceptionObjectArray(_Message):
        def __init__(self, header=None, objects=None, **kwargs):
            super().__init__(header=header or Header(), objects=[] if objects is None else objects, **kwargs)

    class CollisionEvent(_Message):
        pass

    class EgoState(_Message):
        def __init__(self, pose=None, twist=None, speed=0.0, **kwargs):
            super().__init__(pose=pose or Pose(), twist=twist or Twist(), speed=speed, **kwargs)

    class Waypoint(_Message):
        def __init__(self, x=0.0, y=0.0, z=0.0, speed=0.0, lane_id=0, road_id="", **kwargs):
            super().__init__(x=x, y=y, z=z, speed=speed, lane_id=lane_id, road_id=road_id, **kwargs)

    class WaypointArray(_Message):
        def __init__(self, header=None, waypoints=None, **kwargs):
            super().__init__(header=header or Header(), waypoints=[] if waypoints is None else waypoints, **kwargs)

    class ControlCmd(_Message):
        def __init__(self, throttle=0.0, brake=0.0, steer=0.0, gear=0, **kwargs):
            super().__init__(throttle=throttle, brake=brake, steer=steer, gear=gear, **kwargs)

    class MarkerPose(_Message):
        def __init__(self, id=0, pose=None, **kwargs):
            super().__init__(id=id, pose=pose or PoseStamped(), **kwargs)

    class MarkerPoseArray(_Message):
        def __init__(self, header=None, markers=None, **kwargs):
            super().__init__(header=header or Header(), markers=[] if markers is None else markers, **kwargs)

    for cls in (Cluster, ClusterArray, PerceptionObject, PerceptionObjectArray,
                CollisionEvent, EgoState, Waypoint, WaypointArray, ControlCmd,
                MarkerPose, MarkerPoseArray):
        if not hasattr(messages, cls.__name__):
            setattr(messages, cls.__name__, cls)

    course_messages = _module("course_lab_interfaces.msg")
    for cls in (MarkerPose, MarkerPoseArray):
        if not hasattr(course_messages, cls.__name__):
            setattr(course_messages, cls.__name__, cls)

    actions = _module("av_carla_interfaces.action")
    if not hasattr(actions, "Navigate"):
        class Navigate:
            class Goal:
                def __init__(self, target_location=""):
                    self.target_location = target_location

            class Result:
                def __init__(self, success=False, total_time=0.0, avg_speed=0.0):
                    self.success = success
                    self.total_time = total_time
                    self.avg_speed = avg_speed

            class Feedback:
                def __init__(self, status="", progress=0.0):
                    self.status = status
                    self.progress = progress

        actions.Navigate = Navigate

    course_actions = _module("course_lab_interfaces.action")
    if not hasattr(course_actions, "Pipeline"):
        class Pipeline:
            class Goal:
                def __init__(self, recipe_text=""):
                    self.recipe_text = recipe_text

            class Result:
                def __init__(self, success=False, message=""):
                    self.success = success
                    self.message = message

            class Feedback:
                def __init__(self, current_step=0, total_steps=0, step_name=""):
                    self.current_step = current_step
                    self.total_steps = total_steps
                    self.step_name = step_name

        course_actions.Pipeline = Pipeline

    course_services = _module("course_lab_interfaces.srv")
    if not hasattr(course_services, "ReadLabel"):
        class ReadLabel:
            class Request:
                def __init__(self, expected=""):
                    self.expected = expected

            class Response:
                def __init__(self, success=False, message=""):
                    self.success = success
                    self.message = message

        course_services.ReadLabel = ReadLabel
    if not hasattr(course_services, "ValidateRecipe"):
        class ValidateRecipe:
            class Request:
                def __init__(self, recipe_text=""):
                    self.recipe_text = recipe_text

            class Response:
                def __init__(self, success=False, message=""):
                    self.success = success
                    self.message = message

        course_services.ValidateRecipe = ValidateRecipe


def _install_extra_test_interfaces():
    Header = getattr(_module("std_msgs.msg"), "Header")
    Vector3 = getattr(_module("geometry_msgs.msg"), "Vector3")
    sensor_interfaces = _module("sensor_interfaces.msg")
    if not hasattr(sensor_interfaces, "SensorData"):
        class SensorData(_Message):
            def __init__(self, temperature=0.0, humidity=0.0, pressure=0.0, device_id="", **kwargs):
                super().__init__(temperature=temperature, humidity=humidity,
                                 pressure=pressure, device_id=device_id, **kwargs)

        sensor_interfaces.SensorData = SensorData

    example_services = _module("example_interfaces.srv")
    if not hasattr(example_services, "AddTwoInts"):
        class AddTwoInts:
            class Request:
                def __init__(self, a=0, b=0):
                    self.a = a
                    self.b = b

            class Response:
                def __init__(self, sum=0):
                    self.sum = sum

        example_services.AddTwoInts = AddTwoInts

    action_interfaces = _module("action_demo_interfaces.action")
    if not hasattr(action_interfaces, "DoDishes"):
        class DoDishes:
            class Goal:
                def __init__(self, dishwasher_id=0):
                    self.dishwasher_id = dishwasher_id

            class Result:
                def __init__(self, total_dishes_cleaned=0):
                    self.total_dishes_cleaned = total_dishes_cleaned

            class Feedback:
                def __init__(self, percent_complete=0.0):
                    self.percent_complete = percent_complete

        action_interfaces.DoDishes = DoDishes

    rcl_interfaces = _module("rcl_interfaces.msg")
    if not hasattr(rcl_interfaces, "SetParametersResult"):
        class SetParametersResult(_Message):
            def __init__(self, successful=False, reason="", **kwargs):
                super().__init__(successful=successful, reason=reason, **kwargs)

        rcl_interfaces.SetParametersResult = SetParametersResult

    tf2_ros = _module("tf2_ros")
    if not hasattr(tf2_ros, "TransformBroadcaster"):
        class _TransformBroadcaster:
            def __init__(self, *args, **kwargs):
                pass

            def sendTransform(self, transform):
                transforms = transform if isinstance(transform, (list, tuple)) else [transform]
                for item in transforms:
                    _TRANSFORMS[(item.header.frame_id, item.child_frame_id)] = item

        class _Buffer:
            def lookup_transform(self, target_frame, source_frame, time_value):
                key = (target_frame, source_frame)
                if key not in _TRANSFORMS:
                    raise tf2_ros.TransformException(
                        f"No transform from {source_frame} to {target_frame}")
                return _TRANSFORMS[key]

        tf2_ros.TransformBroadcaster = _TransformBroadcaster
        tf2_ros.StaticTransformBroadcaster = _TransformBroadcaster
        tf2_ros.Buffer = _Buffer
        tf2_ros.TransformListener = type("TransformListener", (), {"__init__": lambda self, *a, **k: None})
        tf2_ros.LookupException = type("LookupException", (Exception,), {})
        tf2_ros.TransformException = type("TransformException", (Exception,), {})

    tf2_geometry_msgs = _module("tf2_geometry_msgs")
    if not hasattr(tf2_geometry_msgs, "do_transform_point"):
        tf2_geometry_msgs.do_transform_point = lambda point, transform: point

    nav2_msgs = _module("nav2_msgs.msg")
    if not hasattr(nav2_msgs, "Costmap"):
        nav2_msgs.Costmap = type("Costmap", (_Message,), {})

    nav2_actions = _module("nav2_msgs.action")
    if not hasattr(nav2_actions, "NavigateToPose"):
        nav2_actions.NavigateToPose = type("NavigateToPose", (), {})

    navigator = _module("nav2_simple_commander.robot_navigator")
    if not hasattr(navigator, "BasicNavigator"):
        navigator.BasicNavigator = type("BasicNavigator", (), {})
        navigator.TaskResult = SimpleNamespace(SUCCEEDED=0, CANCELED=1, FAILED=2)

    lifecycle_msgs = _module("lifecycle_msgs.msg")
    if not hasattr(lifecycle_msgs, "State"):
        lifecycle_msgs.State = type(
            "State", (),
            {"PRIMARY_STATE_UNKNOWN": 0, "PRIMARY_STATE_UNCONFIGURED": 1,
             "PRIMARY_STATE_INACTIVE": 2, "PRIMARY_STATE_ACTIVE": 3},
        )
    if not hasattr(lifecycle_msgs, "Transition"):
        lifecycle_msgs.Transition = type(
            "Transition", (),
            {"TRANSITION_CONFIGURE": 1, "TRANSITION_ACTIVATE": 3},
        )

    lifecycle_services = _module("lifecycle_msgs.srv")
    if not hasattr(lifecycle_services, "ChangeState"):
        class ChangeState:
            class Request:
                def __init__(self):
                    self.transition = SimpleNamespace(id=0)

        lifecycle_services.ChangeState = ChangeState
    if not hasattr(lifecycle_services, "GetState"):
        class GetState:
            Request = type("Request", (), {})

        lifecycle_services.GetState = GetState

    ament_packages = _module("ament_index_python.packages")
    if not hasattr(ament_packages, "get_package_prefix"):
        ament_packages.get_package_prefix = lambda package_name: ""
    if not hasattr(ament_packages, "get_package_share_directory"):
        ament_packages.get_package_share_directory = lambda package_name: ""

    moveit_planning = _module("moveit.planning")
    if not hasattr(moveit_planning, "MoveItPy"):
        moveit_planning.MoveItPy = type("MoveItPy", (), {"__init__": lambda self, *a, **k: None})

    moveit_core_state = _module("moveit.core.robot_state")
    if not hasattr(moveit_core_state, "RobotState"):
        class RobotState:
            def __init__(self, *args, **kwargs):
                pass

            def set_to_default_values(self):
                pass

            def set_joint_group_active_positions(self, group, values):
                pass

            def update(self):
                pass

        moveit_core_state.RobotState = RobotState
        moveit_core_state.robotStateToRobotStateMsg = lambda state: state

    moveit_core_trajectory = _module("moveit.core.robot_trajectory")
    if not hasattr(moveit_core_trajectory, "RobotTrajectory"):
        moveit_core_trajectory.RobotTrajectory = type("RobotTrajectory", (), {})

    moveit_msgs = _module("moveit_msgs.msg")

    class _MoveItMessage(_Message):
        ADD = 0
        REMOVE = 1

    class _GripperTranslation(_MoveItMessage):
        def __init__(self, **kwargs):
            super().__init__(
                direction=SimpleNamespace(header=Header(), vector=Vector3()),
                min_distance=0.0,
                desired_distance=0.0,
                **kwargs,
            )

    for name in (
        "RobotState", "CollisionObject", "AttachedCollisionObject", "ObjectColor",
        "PlanningScene", "Grasp", "PlaceLocation",
    ):
        if not hasattr(moveit_msgs, name):
            setattr(moveit_msgs, name, type(name, (_MoveItMessage,), {}))
    if not hasattr(moveit_msgs, "GripperTranslation"):
        moveit_msgs.GripperTranslation = _GripperTranslation
    if not hasattr(moveit_msgs, "MoveItErrorCodes"):
        moveit_msgs.MoveItErrorCodes = type("MoveItErrorCodes", (_Message,), {"SUCCESS": 1})

    moveit_actions = _module("moveit_msgs.action")
    for name in ("Pickup", "Place"):
        if not hasattr(moveit_actions, name):
            setattr(moveit_actions, name, type(name, (), {"Goal": type("Goal", (), {})}))

    moveit_services = _module("moveit_msgs.srv")
    if not hasattr(moveit_services, "GetCartesianPath"):
        class GetCartesianPath:
            class Request:
                def __init__(self):
                    self.header = Header()
                    self.start_state = None
                    self.group_name = ""
                    self.link_name = ""
                    self.waypoints = []
                    self.max_step = 0.0
                    self.jump_threshold = 0.0
                    self.avoid_collisions = False

        moveit_services.GetCartesianPath = GetCartesianPath

    moveit = _module("moveit")
    moveit.__path__ = getattr(moveit, "__path__", [])

    shape_msgs = _module("shape_msgs.msg")
    if not hasattr(shape_msgs, "SolidPrimitive"):
        shape_msgs.SolidPrimitive = type(
            "SolidPrimitive", (_Message,), {"BOX": 1, "SPHERE": 2},
        )

    trajectory_msgs = _module("trajectory_msgs.msg")
    for name in ("JointTrajectory", "JointTrajectoryPoint"):
        if not hasattr(trajectory_msgs, name):
            setattr(trajectory_msgs, name, type(name, (_Message,), {}))

    cv_bridge = _module("cv_bridge")
    if not hasattr(cv_bridge, "CvBridgeError"):
        cv_bridge.CvBridgeError = type("CvBridgeError", (Exception,), {})
    if not hasattr(cv_bridge, "CvBridge"):
        class CvBridge:
            def imgmsg_to_cv2(self, message, desired_encoding="passthrough"):
                return getattr(message, "data", message)

            def cv2_to_imgmsg(self, image, encoding="passthrough"):
                return image

        cv_bridge.CvBridge = CvBridge


if _try_import("rclpy") is None:
    _install_rclpy()

_install_messages()
_install_course_messages()
_install_extra_test_interfaces()
