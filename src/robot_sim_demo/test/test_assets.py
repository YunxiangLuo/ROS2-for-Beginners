import unittest
from pathlib import Path
import xml.etree.ElementTree as ElementTree


class SimulationAssetTest(unittest.TestCase):
    def setUp(self):
        self.package_root = Path(__file__).resolve().parents[1]

    def test_required_files_exist(self):
        required = [
            "launch/gazebo2.launch.py",
            "robot_sim_demo/camera_info_publisher.py",
            "robot_sim_demo/patrol_driver.py",
            "config/gazebo2_bridge.yaml",
            "gui/museum.gui.config",
            "rviz/museum.rviz",
            "urdf/campus_patrol_robot.urdf",
            "wheeltec_robot_urdf/urdf/mini_akm_robot.urdf",
            "worlds/museum.sdf",
            "models/campus_patrol_robot/model.sdf",
            "models/campus_patrol_robot/model.config",
            "models/wheeltec_robot/model.sdf",
            "models/wheeltec_robot/model.config",
            "models/ISCAS_Museum/model.sdf",
            "models/ISCAS_Museum/model.config",
            "models/ISCAS_Museum/meshes/ISCAS_museum.dae",
            "models/ISCAS_Museum/meshes/zd_011.jpg",
            "models/ISCAS_Museum/meshes/zd_021.jpg",
            "models/ISCAS_Museum/meshes/zd_031.jpg",
            "models/ISCAS_Museum/meshes/zd_041.jpg",
            "models/ISCAS_groundplane/model.sdf",
            "models/ISCAS_groundplane/model.config",
            "models/ISCAS_groundplane/meshes/ground_plane.dae",
            "models/ISCAS_groundplane/meshes/ground_plane.obj",
            "models/ISCAS_groundplane/meshes/ground_plane.mtl",
            "models/ISCAS_groundplane/meshes/ISCAS_groundplane.png",
            "models/ISCAS_groundplane/materials/scripts/ISCAS_groundplane.material",
            "models/ISCAS_groundplane/materials/textures/ISCAS_groundplane.png",
            "models/ISCAS_groundplane/materials/textures/flat_heightmap.pgm",
        ]
        for relative_path in required:
            with self.subTest(relative_path=relative_path):
                self.assertTrue((self.package_root / relative_path).is_file())

    def test_sdf_and_urdf_are_well_formed(self):
        for relative_path in (
            "worlds/museum.sdf",
            "models/campus_patrol_robot/model.sdf",
            "models/wheeltec_robot/model.sdf",
            "models/ISCAS_Museum/model.sdf",
            "models/ISCAS_groundplane/model.sdf",
            "urdf/campus_patrol_robot.urdf",
        ):
            with self.subTest(relative_path=relative_path):
                ElementTree.parse(self.package_root / relative_path)

    def test_world_references_bundled_models(self):
        world_text = (self.package_root / "worlds/museum.sdf").read_text(
            encoding="utf-8"
        )
        self.assertIn("model://ISCAS_groundplane", world_text)
        self.assertIn("model://ISCAS_Museum", world_text)

        museum_text = (
            self.package_root / "models/ISCAS_Museum/meshes/ISCAS_museum.dae"
        ).read_text(encoding="utf-8")
        for texture_name in ("zd_011.jpg", "zd_021.jpg", "zd_031.jpg", "zd_041.jpg"):
            self.assertIn(texture_name, museum_text)

    def test_launch_references_this_package_assets(self):
        launch_text = (
            self.package_root / "launch/gazebo2.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('get_package_share_directory("robot_sim_demo")', launch_text)
        self.assertIn('"models" / ROBOT_NAME / "model.sdf"', launch_text)
        self.assertIn('"worlds" / "museum.sdf"', launch_text)
        self.assertIn('"wheeltec_robot_urdf" / "urdf" / "mini_akm_robot.urdf"', launch_text)
        self.assertIn('"config" / "gazebo2_bridge.yaml"', launch_text)
        self.assertNotIn("robot_sim_demo_ros2", launch_text)
        self.assertNotIn("lost_found_ros", launch_text)

    def test_camera_info_node_matches_sensor(self):
        node_text = (
            self.package_root / "robot_sim_demo/camera_info_publisher.py"
        ).read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("width", 320)', node_text)
        self.assertIn('self.declare_parameter("height", 180)', node_text)
        self.assertIn('self.declare_parameter("horizontal_fov", 1.0472)', node_text)
        self.assertIn('self.create_subscription(ClockMessage, "/clock", self.on_clock, 10)', node_text)

    def test_drive_plugin_and_ros_interfaces_are_configured(self):
        model_text = (
            self.package_root / "models/campus_patrol_robot/model.sdf"
        ).read_text(encoding="utf-8")
        bridge_text = (
            self.package_root / "config/gazebo2_bridge.yaml"
        ).read_text(encoding="utf-8")
        launch_text = (
            self.package_root / "launch/gazebo2.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('<static>false</static>', model_text)
        self.assertIn('name="gz::sim::systems::DiffDrive"', model_text)
        self.assertIn('<topic>/cmd_vel</topic>', model_text)
        self.assertIn('ros_topic_name: "/cmd_vel"', bridge_text)
        self.assertIn('ros_topic_name: "/odom"', bridge_text)
        self.assertIn('DeclareLaunchArgument("rviz", default_value="false")', launch_text)
        self.assertIn('executable="patrol_driver"', launch_text)

    def test_wheeltec_model_uses_bundled_meshes(self):
        model_text = (
            self.package_root / "models/wheeltec_robot/model.sdf"
        ).read_text(encoding="utf-8")
        for mesh_name in (
            "base_link.STL",
            "lb_link.STL",
            "rb_link.STL",
            "laser.STL",
            "camera_link.STL",
        ):
            self.assertIn(
                f"model://wheeltec_robot/meshes/{mesh_name}",
                model_text,
            )

    def test_wheeltec_wheels_match_joint_axis(self):
        model_root = ElementTree.parse(
            self.package_root / "models/wheeltec_robot/model.sdf"
        ).getroot()
        model = model_root.find("model")
        self.assertIsNotNone(model)

        for link_name in ("lb_link", "rb_link", "lf_link", "rf_link"):
            with self.subTest(link_name=link_name):
                collision = model.find(f"link[@name='{link_name}']/collision")
                self.assertIsNotNone(collision)
                self.assertEqual("0 0 0 1.5708 0 0", collision.findtext("pose"))
                self.assertEqual("0.033", collision.findtext("geometry/cylinder/radius"))

        for joint_name in ("lb_joint", "rb_joint", "lf_point", "rf_point"):
            with self.subTest(joint_name=joint_name):
                self.assertEqual(
                    "0 1 0",
                    model.findtext(f"joint[@name='{joint_name}']/axis/xyz"),
                )

        plugin = model.find("plugin")
        self.assertEqual(
            ["lb_joint", "lf_point"],
            [element.text for element in plugin.findall("left_joint")],
        )
        self.assertEqual(
            ["rb_joint", "rf_point"],
            [element.text for element in plugin.findall("right_joint")],
        )

        self.assertEqual(
            "10.0",
            model.findtext("plugin/max_linear_velocity"),
        )
