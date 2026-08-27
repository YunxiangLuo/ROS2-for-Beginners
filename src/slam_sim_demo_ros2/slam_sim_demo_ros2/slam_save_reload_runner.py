import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy

from .slam_map_runner import command_for_elapsed, count_known_cells, planar_distance


class ReloadedMapWatcher(Node):
    def __init__(self) -> None:
        super().__init__("slam_save_reload_runner")
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.handle_odom, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, "/map", self.handle_map, 10)
        self.reloaded_map = None
        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        self.reloaded_subscription = self.create_subscription(
            OccupancyGrid,
            "/reloaded_map",
            self.handle_reloaded_map,
            transient_qos,
        )
        self.start_pose = None
        self.latest_pose = None
        self.first_map_known_cells = None
        self.max_known_cells = 0
        self.map_updates = 0
        self.last_map_stamp = None
        self.reloaded_map_known_cells = None

    def handle_odom(self, message: Odometry) -> None:
        pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if self.start_pose is None:
            self.start_pose = pose
        self.latest_pose = pose

    def handle_map(self, message: OccupancyGrid) -> None:
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp != self.last_map_stamp:
            self.map_updates += 1
            self.last_map_stamp = stamp
        known_cells = count_known_cells(list(message.data))
        if self.first_map_known_cells is None:
            self.first_map_known_cells = known_cells
        self.max_known_cells = max(self.max_known_cells, known_cells)

    def handle_reloaded_map(self, message: OccupancyGrid) -> None:
        self.reloaded_map = message
        self.reloaded_map_known_cells = count_known_cells(list(message.data))

    def ready(self) -> bool:
        return self.latest_pose is not None and self.first_map_known_cells is not None

    def odom_distance(self) -> float:
        if self.start_pose is None or self.latest_pose is None:
            return 0.0
        return planar_distance(self.start_pose, self.latest_pose)

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.command_publisher.publish(twist)

    def publish_stop(self) -> None:
        self.publish_command(0.0, 0.0)

    def slam_map_ready(self) -> bool:
        if self.first_map_known_cells is None:
            return False
        return (
            self.map_updates >= 2
            and self.odom_distance() > 0.15
            and self.max_known_cells - self.first_map_known_cells > 20
        )


def wait_for_state(node: Node, get_state_client, target_state: int, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        future = get_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        response = future.result()
        if response is not None and int(response.current_state.id) == target_state:
            return True
        time.sleep(0.1)
    return False


def change_state(node: Node, change_state_client, transition_id: int) -> None:
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = change_state_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)


def main() -> None:
    rclpy.init()
    node = ReloadedMapWatcher()
    use_sim_time = bool(node.get_parameter("use_sim_time").value)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    overall_start = time.time()
    motion_start = None
    map_server_process = None
    try:
        while time.time() - overall_start < 60.0:
            executor.spin_once(timeout_sec=0.1)
            if not node.ready():
                continue
            if motion_start is None:
                motion_start = time.time()
            linear_x, angular_z = command_for_elapsed(time.time() - motion_start)
            node.publish_command(linear_x, angular_z)
            if node.slam_map_ready():
                break
        else:
            raise RuntimeError("slam_toolbox did not publish a growing map before save/reload check")

        node.publish_stop()
        with tempfile.TemporaryDirectory(prefix="slam_map_") as temp_dir:
            base_path = Path(temp_dir) / "saved_map"
            map_saver_executable = shutil.which("map_saver_cli")
            if map_saver_executable is None:
                prefix = Path(get_package_prefix("nav2_map_server"))
                candidate = prefix / "lib" / "nav2_map_server" / "map_saver_cli"
                if candidate.is_file():
                    map_saver_executable = str(candidate)
            if map_saver_executable is None:
                raise RuntimeError("Could not locate nav2_map_server map_saver_cli executable")

            save_process = subprocess.run(
                [
                    map_saver_executable,
                    "-t",
                    "/map",
                    "-f",
                    str(base_path),
                    "--fmt",
                    "pgm",
                    "--mode",
                    "trinary",
                    "--ros-args",
                    "-p",
                    f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=15.0,
            )
            if save_process.returncode != 0:
                raise RuntimeError(f"map_saver_cli failed with exit code {save_process.returncode}")

            yaml_path = base_path.with_suffix(".yaml")
            pgm_path = base_path.with_suffix(".pgm")
            if not yaml_path.is_file() or not pgm_path.is_file():
                raise RuntimeError("Saved map files were not generated")

            map_server_executable = shutil.which("map_server")
            if map_server_executable is None:
                prefix = Path(get_package_prefix("nav2_map_server"))
                candidate = prefix / "lib" / "nav2_map_server" / "map_server"
                if candidate.is_file():
                    map_server_executable = str(candidate)
            if map_server_executable is None:
                raise RuntimeError("Could not locate nav2_map_server map_server executable")

            map_server_process = subprocess.Popen(
                [
                    map_server_executable,
                    "--ros-args",
                    "-r",
                    "__node:=reloaded_map_server",
                    "-r",
                    "/map:=/reloaded_map",
                    "-p",
                    f"yaml_filename:={yaml_path}",
                    "-p",
                    f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            reload_change_client = node.create_client(ChangeState, "/reloaded_map_server/change_state")
            reload_state_client = node.create_client(GetState, "/reloaded_map_server/get_state")
            if not reload_change_client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("Timed out waiting for reloaded_map_server lifecycle services")
            if not reload_state_client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("Timed out waiting for reloaded_map_server state service")

            change_state(node, reload_change_client, Transition.TRANSITION_CONFIGURE)
            if not wait_for_state(node, reload_state_client, State.PRIMARY_STATE_INACTIVE, 10.0):
                raise RuntimeError("reloaded_map_server did not reach inactive state")
            change_state(node, reload_change_client, Transition.TRANSITION_ACTIVATE)
            if not wait_for_state(node, reload_state_client, State.PRIMARY_STATE_ACTIVE, 10.0):
                raise RuntimeError("reloaded_map_server did not reach active state")

            deadline = time.time() + 10.0
            while time.time() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if node.reloaded_map is not None:
                    break
            else:
                raise RuntimeError("Timed out waiting for /reloaded_map after reloading saved YAML")

            if node.reloaded_map_known_cells is None or node.reloaded_map_known_cells <= 0:
                raise RuntimeError("Reloaded map contained no known cells")

            print("slam-map-saved-and-reloaded")
    finally:
        node.publish_stop()
        if map_server_process is not None and map_server_process.poll() is None:
            map_server_process.send_signal(signal.SIGINT)
            try:
                map_server_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                map_server_process.kill()
                map_server_process.wait(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
