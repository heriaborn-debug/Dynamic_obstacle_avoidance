#!/usr/bin/env python3

import importlib.util
import sys

import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray


def load_planner(path):
    spec = importlib.util.spec_from_file_location("m20_first_planner_v2_impl", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FirstPlannerV2


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_fallback_order.py PLANNER_SCRIPT")

    planner_type = load_planner(sys.argv[1])
    rclpy.init()
    planner = planner_type()
    try:
        assert planner.fallback_stages == [
            (1.0, 4.0),
            (0.75, 3.0),
            (0.75, 2.5),
            (0.75, 2.0),
            (0.75, 1.5),
            (0.75, 1.0),
        ]
        shape = (len(planner.fallback_stages), planner.path_array_size)
        goal = (3.0, 0.0, 0.0, 3.0)

        planner.dynamic_costs = np.zeros(shape, dtype=np.float64)
        planner.dynamic_costs[0, :] = planner.dynamic_block_cost
        planner.dynamic_cost_received_time = planner.get_clock().now()
        selected = planner._select_path(goal)
        assert selected[0] is not None
        assert selected[3] == 1
        assert planner.selection_mode == "path_scale"

        planner.dynamic_costs[:, :] = planner.dynamic_block_cost
        planner.dynamic_costs[-1, :] = 0.0
        planner.dynamic_cost_received_time = planner.get_clock().now()
        selected = planner._select_path(goal)
        assert selected[0] is not None
        assert selected[3] == len(planner.fallback_stages) - 1
        assert planner.selection_mode == "path_range"

        planner.dynamic_costs[:, :] = planner.dynamic_block_cost
        planner.dynamic_cost_received_time = planner.get_clock().now()
        planner.dynamic_escape_costs = np.zeros(
            planner.path_array_size, dtype=np.float64
        )
        planner.dynamic_escape_received_time = planner.get_clock().now()
        selected = planner._select_path(goal)
        assert selected[0] is not None
        assert planner.selection_mode == "escape"

        first_escape_path = selected[0]
        planner.dynamic_escape_costs[:] = np.inf
        planner.dynamic_escape_received_time = planner.get_clock().now()
        planner.stop_position_intruded = True
        planner.intrusion_latched = True
        planner.stop_position_first_collision_time = 0.8
        planner.reachability_received_time = planner.get_clock().now()
        selected = planner._select_path(goal)
        assert selected[0] is None
        assert planner.selection_mode == "intrusion_blocked"

        planner.dynamic_escape_costs[:] = 0.0
        planner.dynamic_escape_costs[first_escape_path] = np.inf
        planner.dynamic_escape_received_time = planner.get_clock().now()
        selected = planner._select_path(goal)
        assert selected[0] is not None
        assert selected[0] != first_escape_path
        assert planner.selection_mode == "escape"
        assert planner._stop_intrusion_is_fresh()

        reachability = Float32MultiArray()
        reachability.data = [
            0.0,
            0.0,
            0.0,
            1.0,
            1.0,
            0.0,
            -0.1,
            0.8,
            0.1,
            4641.0,
        ]
        planner._on_true_robot_reachability(reachability)
        assert planner.intrusion_latched

        reachability.data[3] = 0.0
        reachability.data[4] = 0.0
        reachability.data[5] = 1.0
        planner._on_true_robot_reachability(reachability)
        assert planner.intrusion_latched
        assert planner.intrusion_release_clear_frames == 0

        reachability.data[5] = 0.0
        for _ in range(planner.intrusion_release_confirmation_frames - 1):
            planner._on_true_robot_reachability(reachability)
            assert planner.intrusion_latched
        planner._on_true_robot_reachability(reachability)
        assert not planner.intrusion_latched
    finally:
        planner.destroy_node()
        rclpy.shutdown()

    print("FIRST V2 fallback order: PASS")


if __name__ == "__main__":
    main()
