import unittest

from slam_sim_demo_ros2.slam_map_runner import command_for_elapsed, count_known_cells, planar_distance


class SlamMapRunnerUnitTest(unittest.TestCase):
    def test_count_known_cells_ignores_unknown_values(self):
        self.assertEqual(count_known_cells([-1, 0, 50, 100, -1]), 3)

    def test_planar_distance(self):
        self.assertAlmostEqual(planar_distance((0.0, 0.0), (3.0, 4.0)), 5.0)

    def test_command_sequence_cycles(self):
        self.assertEqual(command_for_elapsed(0.5), (0.18, 0.0))
        self.assertEqual(command_for_elapsed(9.0), (0.0, 0.5))
        self.assertEqual(command_for_elapsed(15.0), (0.18, 0.0))
        self.assertEqual(command_for_elapsed(20.5), (0.18, 0.0))
