from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.path_planner import APFPlanner, PurePursuitPlanner


class PurePursuitPlannerTests(unittest.TestCase):
    def test_lookahead_uses_ordered_remaining_path(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=50.0)
        path = np.array([
            [0.0, 500.0],
            [25.0, 500.0],
            [50.0, 500.0],
            [75.0, 500.0],
        ])

        lookahead = planner._lookahead_point(0.0, 480.0, path)

        self.assertEqual(tuple(lookahead), (50.0, 500.0))

    def test_compute_velocity_is_straight_on_for_aligned_target(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=100.0, max_angular=1.0)

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[100.0, 0.0]]),
            max_linear=100.0,
        )

        self.assertAlmostEqual(linear, 100.0, places=3)
        self.assertAlmostEqual(angular, 0.0, places=3)

    def test_compute_velocity_slows_and_turns_for_corner(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=100.0, max_angular=1.0)

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[0.0, 100.0]]),
            max_linear=100.0,
        )

        self.assertAlmostEqual(linear, 0.0, places=3)
        self.assertGreater(angular, 0.0)
        self.assertLessEqual(angular, 1.0)


class APFPlannerTests(unittest.TestCase):
    def test_navigate_straight_ahead_no_obstacles(self) -> None:
        planner = APFPlanner(
            max_linear=200.0, max_angular=2.0,
            attraction_gain=1.0, repulsion_gain=500.0,
            repulsion_range=300.0, goal_tolerance=20.0, heading_gain=2.0,
        )
        empty = np.empty((0, 2))

        linear, angular = planner.navigate_to_goal((0, 0, 0), (500, 0), empty)

        self.assertGreater(linear, 0.0)
        self.assertAlmostEqual(angular, 0.0, places=2)

    def test_navigate_turns_left_toward_goal(self) -> None:
        planner = APFPlanner(
            max_linear=200.0, max_angular=2.0,
            attraction_gain=1.0, repulsion_gain=500.0,
            repulsion_range=300.0, goal_tolerance=20.0, heading_gain=2.0,
        )
        empty = np.empty((0, 2))

        _, angular = planner.navigate_to_goal((0, 0, 0), (0, 500), empty)

        self.assertGreater(angular, 0.0)

    def test_navigate_stops_at_goal(self) -> None:
        planner = APFPlanner(goal_tolerance=20.0)
        empty = np.empty((0, 2))

        linear, angular = planner.navigate_to_goal((495, 0, 0), (500, 0), empty)

        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, 0.0)

    def test_obstacle_ahead_blocks_forward_command(self) -> None:
        planner = APFPlanner(
            max_linear=200.0, max_angular=2.0,
            attraction_gain=1.0, repulsion_gain=800.0,
            repulsion_range=400.0, goal_tolerance=20.0, heading_gain=2.0,
        )
        obstacle_line = np.array([
            [200.0, -120.0],
            [200.0, -80.0],
            [200.0, -40.0],
            [200.0, 0.0],
            [200.0, 40.0],
            [200.0, 80.0],
            [200.0, 120.0],
        ])

        linear, angular = planner.navigate_to_goal((0.0, 0.0, 0.0), (500.0, 0.0), obstacle_line)

        self.assertLessEqual(linear, 1e-6)
        self.assertGreater(abs(angular), 0.0)

    def test_disk_obstacle_ahead_blocks_forward_command(self) -> None:
        planner = APFPlanner(
            max_linear=200.0, max_angular=2.0,
            attraction_gain=1.0, repulsion_gain=800.0,
            repulsion_range=400.0, goal_tolerance=20.0, heading_gain=2.0,
        )
        obstacle_disks = np.array([
            [220.0, 0.0, 75.0],
            [220.0, 120.0, 75.0],
        ])

        linear, angular = planner.navigate_to_goal((0.0, 0.0, 0.0), (500.0, 0.0), obstacle_disks)

        self.assertLessEqual(linear, 1e-6)
        self.assertGreater(abs(angular), 0.0)


if __name__ == "__main__":
    unittest.main()
