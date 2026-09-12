#!/usr/bin/env python3

import math
import os
from collections import defaultdict

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from drdds.msg import NavCmd
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_scalar_config(path):
    values = {}
    if not path or not os.path.exists(path):
        return values
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()
            if not key or not value:
                continue
            lowered = value.lower()
            if lowered in ("true", "false"):
                values[key] = lowered == "true"
                continue
            try:
                if any(ch in value for ch in ".eE"):
                    values[key] = float(value)
                else:
                    values[key] = int(value)
            except ValueError:
                values[key] = value
    return values


def config_float(values, key, default):
    return float(values.get(key, default))


def config_int(values, key, default):
    return int(values.get(key, default))


class FirstPlannerV2(Node):
    def __init__(self):
        super().__init__("m20_first_planner_v2")
        default_path = os.path.join(
            get_package_share_directory("m20_gazebo"), "first_paths"
        )
        self.declare_parameter("path_folder", default_path)
        self.declare_parameter(
            "localplanner_config", os.path.join(default_path, "localplanner.yaml")
        )
        self.declare_parameter("odom_topic", "/m20/ground_truth")
        self.declare_parameter("goal_topic", "/local_goal")
        self.declare_parameter("goal_alias_topics", [""])
        self.declare_parameter("cloud_topic", "/m20/airy_obstacle_points")
        self.declare_parameter("cmd_topic", "/NAV_CMD")
        self.declare_parameter("local_plan_topic", "/m20/v2/local_plan")
        self.declare_parameter("track_path_topic", "/m20/v2/track_path")
        self.declare_parameter(
            "selected_prediction_markers_topic",
            "/m20/v2/first_selected_prediction_markers",
        )
        self.declare_parameter("selected_prediction_horizon", 3.0)
        self.declare_parameter("selected_prediction_step", 0.1)
        self.declare_parameter("local_scans_topic", "/local_scans")
        self.declare_parameter("local_obstacle_grid_topic", "/m20/v2/local_obstacle_grid")
        self.declare_parameter("planner_stage_topic", "/m20/v2/planner_stage")
        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("yaw_tolerance", 0.35)
        self.declare_parameter("require_goal_yaw", False)
        self.declare_parameter("max_goal_range", 3.0)
        self.declare_parameter("obstacle_range", 4.8)
        self.declare_parameter("self_clear_x", 0.68)
        self.declare_parameter("self_clear_y", 0.42)
        self.declare_parameter("hard_radius", 0.42)
        self.declare_parameter("soft_radius", 0.82)
        self.declare_parameter("grid_resolution", 0.16)
        self.declare_parameter("goal_distance_weight", 1.0)
        self.declare_parameter("goal_heading_weight", 0.5)
        self.declare_parameter("speed_weight", 0.2)
        self.declare_parameter("reverse_penalty", 0.5)
        self.declare_parameter("lateral_penalty", 0.15)
        self.declare_parameter("use_correspondence_collision", True)
        self.declare_parameter("publish_zero_without_goal", True)
        self.declare_parameter("reset_navigation_topic", "/m20/reset_navigation")
        self.declare_parameter(
            "dynamic_cost_topic", "/m20/v2/first_dynamic_costs"
        )
        self.declare_parameter("dynamic_cost_timeout", 0.35)
        self.declare_parameter("dynamic_block_cost", 1000000.0)
        self.declare_parameter("require_dynamic_input", True)
        self.declare_parameter("dynamic_release_confirmation_frames", 3)
        self.declare_parameter("intrusion_release_confirmation_frames", 4)
        self.declare_parameter(
            "dynamic_escape_cost_topic", "/m20/v2/first_dynamic_escape_costs"
        )
        self.declare_parameter(
            "dynamic_obstacles_topic", "/m20/v2/first_dynamic_obstacles_local"
        )
        self.declare_parameter(
            "true_robot_speed_limits_topic",
            "/m20/v2/true_robot_speed_limits",
        )
        self.declare_parameter(
            "true_robot_reachability_topic",
            "/m20/v2/true_robot_reachability",
        )
        self.declare_parameter("dynamic_point_mask_padding", 0.15)
        self.declare_parameter("escape_min_translation_speed", 0.15)
        self.declare_parameter("escape_max_translation_speed", 1.0)
        self.declare_parameter("escape_goal_cost_blend", 0.05)
        self.declare_parameter("path_scale", 1.0)
        self.declare_parameter("min_path_scale", 0.75)
        self.declare_parameter("path_scale_step", 0.25)
        self.declare_parameter("path_range", 4.0)
        self.declare_parameter("min_path_range", 1.0)
        self.declare_parameter("path_range_step", 0.5)
        self.declare_parameter("path_switch_penalty", 1.25)

        self.path_folder = str(self.get_parameter("path_folder").value)
        self.config_path = str(self.get_parameter("localplanner_config").value)
        self.config = parse_scalar_config(self.config_path)
        self.goal_tolerance = config_float(
            self.config, "xy_tolerance", self.get_parameter("goal_tolerance").value
        )
        self.yaw_tolerance = config_float(
            self.config, "yaw_tolerance", self.get_parameter("yaw_tolerance").value
        )
        self.max_goal_range = config_float(
            self.config, "local_point_dis", self.get_parameter("max_goal_range").value
        )
        self.obstacle_range = float(self.get_parameter("obstacle_range").value)
        self.self_clear_x = float(self.get_parameter("self_clear_x").value)
        self.self_clear_y = float(self.get_parameter("self_clear_y").value)
        self.hard_radius = float(self.get_parameter("hard_radius").value)
        self.soft_radius = float(self.get_parameter("soft_radius").value)
        self.grid_resolution = float(self.get_parameter("grid_resolution").value)
        self.goal_distance_weight = config_float(
            self.config,
            "weight_goal",
            self.get_parameter("goal_distance_weight").value,
        )
        self.goal_heading_weight = config_float(
            self.config, "weight_yaw", self.get_parameter("goal_heading_weight").value
        )
        self.speed_weight = config_float(
            self.config, "weight_spdy", self.get_parameter("speed_weight").value
        )
        self.weight_ob1 = config_float(self.config, "weight_ob1", 0.6)
        self.weight_ob2 = config_float(self.config, "weight_ob2", 0.8)
        self.weight_ob3 = config_float(self.config, "weight_ob3", 1.0)
        self.obstacle_height_threshold = config_float(
            self.config, "obstacleHeightThre", 0.10
        )
        self.point_per_path_threshold = config_int(self.config, "pointPerPathThre", 1)
        self.grid_voxel_size = config_float(self.config, "grid_voxel_size", 0.05)
        self.grid_voxel_offset_x = config_float(
            self.config, "grid_voxel_offset_x", -1.475
        )
        self.grid_voxel_offset_y = config_float(
            self.config, "grid_voxel_offset_y", -1.975
        )
        self.grid_voxel_num_x = config_int(self.config, "grid_voxel_num_x", 90)
        self.grid_voxel_num_y = config_int(self.config, "grid_voxel_num_y", 80)
        self.grid_occupancy_threshold = config_int(self.config, "grid_occu_num_", 2)
        self.max_speed_x = config_float(self.config, "maxSpeedX", 1.5)
        self.max_speed_y = config_float(self.config, "maxSpeedY", 0.6)
        self.max_theta = config_float(self.config, "maxTheta", 1.0)
        self.reverse_penalty = float(self.get_parameter("reverse_penalty").value)
        self.lateral_penalty = float(self.get_parameter("lateral_penalty").value)
        self.use_correspondence_collision = bool(
            self.get_parameter("use_correspondence_collision").value
        )
        self.publish_zero_without_goal = bool(
            self.get_parameter("publish_zero_without_goal").value
        )
        self.require_goal_yaw = bool(self.get_parameter("require_goal_yaw").value)
        self.dynamic_cost_timeout = float(
            self.get_parameter("dynamic_cost_timeout").value
        )
        self.dynamic_block_cost = float(
            self.get_parameter("dynamic_block_cost").value
        )
        self.require_dynamic_input = bool(
            self.get_parameter("require_dynamic_input").value
        )
        self.dynamic_release_confirmation_frames = int(
            self.get_parameter("dynamic_release_confirmation_frames").value
        )
        if self.dynamic_release_confirmation_frames < 1:
            raise ValueError("dynamic_release_confirmation_frames must be positive")
        self.intrusion_release_confirmation_frames = int(
            self.get_parameter("intrusion_release_confirmation_frames").value
        )
        if self.intrusion_release_confirmation_frames < 1:
            raise ValueError(
                "intrusion_release_confirmation_frames must be positive"
            )
        self.dynamic_point_mask_padding = float(
            self.get_parameter("dynamic_point_mask_padding").value
        )
        self.escape_min_translation_speed = float(
            self.get_parameter("escape_min_translation_speed").value
        )
        self.escape_max_translation_speed = float(
            self.get_parameter("escape_max_translation_speed").value
        )
        self.escape_goal_cost_blend = float(
            self.get_parameter("escape_goal_cost_blend").value
        )
        self.path_scale = float(self.get_parameter("path_scale").value)
        self.min_path_scale = float(self.get_parameter("min_path_scale").value)
        self.path_scale_step = float(self.get_parameter("path_scale_step").value)
        self.path_range = float(self.get_parameter("path_range").value)
        self.min_path_range = float(self.get_parameter("min_path_range").value)
        self.path_range_step = float(self.get_parameter("path_range_step").value)
        self.path_switch_penalty = float(
            self.get_parameter("path_switch_penalty").value
        )
        self.fallback_stages = self._build_fallback_stages()
        self.selected_prediction_horizon = float(
            self.get_parameter("selected_prediction_horizon").value
        )
        self.selected_prediction_step = float(
            self.get_parameter("selected_prediction_step").value
        )

        self.paths = {}
        self.ends = {}
        self._load_paths()
        self._build_path_arrays()
        self._build_stage_geometry()
        self.correspondence_layers = []
        self._load_correspondences()

        self.odom = None
        self.goal = None
        self.goal_source = ""
        self.obstacles = []
        self.obstacle_count = 0
        self.obstacle_grid = defaultdict(list)
        self.occupied_voxels = set()
        self.occupied_voxel_centers = np.empty((0, 2), dtype=np.float64)
        self.blocked_paths = np.zeros(self.path_array_size, dtype=np.int32)
        self.path_penalty1 = np.zeros(self.path_array_size, dtype=np.int32)
        self.path_penalty2 = np.zeros(self.path_array_size, dtype=np.int32)
        self.path_penalty3 = np.zeros(self.path_array_size, dtype=np.int32)
        self.stage_blocked_paths = []
        self.stage_path_penalty1 = []
        self.stage_path_penalty2 = []
        self.stage_path_penalty3 = []
        self._rebuild_path_penalties()
        self.selected_path_id = None
        self.last_diagnostic_time = None
        self.dynamic_costs = None
        self.dynamic_cost_received_time = None
        self.dynamic_cost_generation = 0
        self.dynamic_cost_active = False
        self.dynamic_blocked_count = 0
        self.dynamic_cost_age = math.inf
        self.dynamic_escape_costs = None
        self.dynamic_escape_received_time = None
        self.dynamic_obstacles = np.empty((0, 3), dtype=np.float64)
        self.dynamic_obstacles_received_time = None
        self.true_robot_speed_limits = None
        self.true_robot_speed_limits_received_time = None
        self.stop_position_intruded = False
        self.scorer_threat_latched = False
        self.prediction_coasting = False
        self.intrusion_latched = False
        self.intrusion_release_clear_frames = 0
        self.stop_position_first_collision_time = math.inf
        self.reachability_prediction_age = math.inf
        self.reachability_received_time = None
        self.dynamic_masked_point_count = 0
        self.dynamic_stop_latched = False
        self.dynamic_release_clear_frames = 0
        self.dynamic_release_last_generation = -1
        self.selection_mode = "normal"
        self.selected_stage_index = 0
        self.selected_scale = self.fallback_stages[0][0]
        self.selected_range = self.fallback_stages[0][1]
        self.stage_blocked_counts = [0] * len(self.fallback_stages)

        self.cmd_pub = self.create_publisher(
            NavCmd, str(self.get_parameter("cmd_topic").value), 10
        )
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("local_plan_topic").value), 10
        )
        self.track_path_pub = self.create_publisher(
            Path, str(self.get_parameter("track_path_topic").value), 10
        )
        self.selected_prediction_markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("selected_prediction_markers_topic").value),
            10,
        )
        self.local_obstacle_grid_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("local_obstacle_grid_topic").value),
            10,
        )
        self.planner_stage_pub = self.create_publisher(
            String, str(self.get_parameter("planner_stage_topic").value), 10
        )
        self.local_scans_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("local_scans_topic").value), 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            lambda message: self._on_goal(message, str(self.get_parameter("goal_topic").value)),
            10,
        )
        goal_topics = [
            str(topic)
            for topic in self.get_parameter("goal_alias_topics").value
            if str(topic)
        ]
        for topic in goal_topics:
            self.create_subscription(
                PoseStamped,
                topic,
                lambda message, topic=topic: self._on_goal(message, topic),
                10,
            )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("reset_navigation_topic").value),
            self._on_reset_navigation,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("dynamic_cost_topic").value),
            self._on_dynamic_costs,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("dynamic_escape_cost_topic").value),
            self._on_dynamic_escape_costs,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("dynamic_obstacles_topic").value),
            self._on_dynamic_obstacles,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("true_robot_speed_limits_topic").value),
            self._on_true_robot_speed_limits,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("true_robot_reachability_topic").value),
            self._on_true_robot_reachability,
            10,
        )
        frequency = float(self.get_parameter("control_frequency").value)
        self.create_timer(1.0 / max(1.0, frequency), self._control_update)
        self.get_logger().info(
            "M20 FIRST V2 loaded %d candidate paths from %s, config=%s, cmd=%s, goals=%s"
            % (
                len(self.ends),
                self.path_folder,
                self.config_path if self.config else "defaults",
                str(self.get_parameter("cmd_topic").value),
                [str(self.get_parameter("goal_topic").value)] + goal_topics,
            )
        )
        self.get_logger().info(
            "FIRST V2 correspondence collision %s, layers=%d, fallback=%s"
            % (
                "enabled" if self.use_correspondence_collision else "disabled",
                len(self.correspondence_layers),
                ", ".join(
                    "scale=%.2f/range=%.2f" % stage
                    for stage in self.fallback_stages
                ),
            )
        )

    def _build_fallback_stages(self):
        if (
            self.path_scale <= 0.0
            or self.min_path_scale <= 0.0
            or self.path_scale_step <= 0.0
            or self.path_range <= 0.0
            or self.min_path_range <= 0.0
            or self.path_range_step <= 0.0
            or self.min_path_scale > self.path_scale
            or self.min_path_range > self.path_range
        ):
            raise ValueError("Invalid FIRST V2 path scale/range fallback parameters")

        stages = [(self.path_scale, self.path_range)]
        scale = self.path_scale
        path_range = self.path_range
        while scale >= self.min_path_scale and path_range >= self.min_path_range:
            if scale >= self.min_path_scale + self.path_scale_step - 1.0e-9:
                scale = max(self.min_path_scale, scale - self.path_scale_step)
                path_range = self.path_range * scale / self.path_scale
            else:
                path_range -= self.path_range_step
            if path_range < self.min_path_range - 1.0e-9:
                break
            stage = (round(scale, 6), round(path_range, 6))
            if stage != stages[-1]:
                stages.append(stage)
        return stages

    def _on_dynamic_costs(self, message):
        costs = np.asarray(message.data, dtype=np.float64)
        expected = len(self.fallback_stages) * self.path_array_size
        if costs.size != expected:
            self.get_logger().error(
                "FIRST V2 dynamic cost size mismatch: received=%d expected=%d"
                % (costs.size, expected)
            )
            return
        self.dynamic_costs = costs.reshape(
            (len(self.fallback_stages), self.path_array_size)
        )
        self.dynamic_cost_received_time = self.get_clock().now()
        self.dynamic_cost_generation += 1

    def _on_dynamic_escape_costs(self, message):
        costs = np.asarray(message.data, dtype=np.float64)
        if costs.size != self.path_array_size:
            self.get_logger().error(
                "FIRST V2 dynamic escape cost size mismatch: received=%d expected=%d"
                % (costs.size, self.path_array_size)
            )
            return
        self.dynamic_escape_costs = costs
        self.dynamic_escape_received_time = self.get_clock().now()

    def _on_dynamic_obstacles(self, message):
        values = np.asarray(message.data, dtype=np.float64)
        if values.size == 0:
            self.dynamic_obstacles = np.empty((0, 3), dtype=np.float64)
            self.dynamic_obstacles_received_time = self.get_clock().now()
            return
        if values.size % 3 != 0:
            self.get_logger().error(
                "FIRST dynamic obstacle array is not x/y/radius triples: size=%d"
                % values.size
            )
            return
        self.dynamic_obstacles = values.reshape((-1, 3))
        self.dynamic_obstacles_received_time = self.get_clock().now()

    def _on_true_robot_speed_limits(self, message):
        values = np.asarray(message.data, dtype=np.float64)
        if values.size != 4:
            self.get_logger().error(
                "FIRST V2 true-robot speed limit size mismatch: received=%d expected=4"
                % values.size
            )
            return
        self.true_robot_speed_limits = values
        self.true_robot_speed_limits_received_time = self.get_clock().now()

    def _on_true_robot_reachability(self, message):
        values = np.asarray(message.data, dtype=np.float64)
        if values.size != 10:
            self.get_logger().error(
                "FIRST V2 reachability size mismatch: received=%d expected=10"
                % values.size
            )
            return
        self.stop_position_intruded = bool(values[3] > 0.5)
        self.scorer_threat_latched = bool(values[4] > 0.5)
        self.prediction_coasting = bool(values[5] > 0.5)
        self.stop_position_first_collision_time = float(values[7])
        self.reachability_prediction_age = float(values[8])
        self.reachability_received_time = self.get_clock().now()
        if self.stop_position_intruded or self.scorer_threat_latched:
            self.intrusion_latched = True
            self.intrusion_release_clear_frames = 0
        elif self.intrusion_latched and not self.prediction_coasting:
            self.intrusion_release_clear_frames += 1
            if (
                self.intrusion_release_clear_frames
                >= self.intrusion_release_confirmation_frames
            ):
                self.intrusion_latched = False
                self.intrusion_release_clear_frames = 0

    def _stop_intrusion_is_fresh(self):
        return self.intrusion_latched

    def _stream_is_fresh(self, received_time):
        if received_time is None:
            return False
        age = (self.get_clock().now() - received_time).nanoseconds * 1.0e-9
        return age <= self.dynamic_cost_timeout

    def _dynamic_caused_no_solution(self):
        if not self.dynamic_cost_active or self.dynamic_costs is None:
            return False
        ids = self.path_ids
        for stage_index in range(len(self.fallback_stages)):
            static_blocked = (
                self.stage_blocked_paths[stage_index][ids]
                >= self.point_per_path_threshold
            )
            dynamic_blocked = (
                self.dynamic_costs[stage_index, ids] >= self.dynamic_block_cost
            )
            if np.any(~static_blocked) and not np.any(
                ~(np.logical_or(static_blocked, dynamic_blocked))
            ):
                return True
        return False

    def _load_paths(self):
        path_file = os.path.join(self.path_folder, "pathList.ply")
        end_file = os.path.join(self.path_folder, "path_end.ply")
        with open(path_file, "r", encoding="utf-8") as handle:
            for line in handle:
                parts = line.split()
                if len(parts) != 4:
                    continue
                x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]))
                path_id = int(parts[3])
                self.paths.setdefault(path_id, []).append((x, y, z))

        with open(end_file, "r", encoding="utf-8") as handle:
            for line in handle:
                parts = line.split()
                if len(parts) != 8:
                    continue
                path_id = int(parts[7])
                self.ends[path_id] = {
                    "end": (float(parts[0]), float(parts[1]), float(parts[2])),
                    "yaw": float(parts[3]),
                    "vel": (float(parts[4]), float(parts[5]), float(parts[6])),
                }

        missing = [path_id for path_id in self.ends if path_id not in self.paths]
        if missing:
            raise RuntimeError("Missing sampled points for FIRST paths: %s" % missing[:5])

    def _build_path_arrays(self):
        self.path_ids = np.asarray(sorted(self.ends), dtype=np.int32)
        self.path_array_size = int(self.path_ids[-1]) + 1
        self.path_end_x = np.asarray(
            [self.ends[int(path_id)]["end"][0] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_end_y = np.asarray(
            [self.ends[int(path_id)]["end"][1] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_end_yaw = np.asarray(
            [self.ends[int(path_id)]["yaw"] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_vel_x = np.asarray(
            [self.ends[int(path_id)]["vel"][0] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_vel_y = np.asarray(
            [self.ends[int(path_id)]["vel"][1] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_vel_yaw = np.asarray(
            [self.ends[int(path_id)]["vel"][2] for path_id in self.path_ids],
            dtype=np.float64,
        )
        self.path_sample_counts = np.asarray(
            [max(1, len(self.paths[int(path_id)])) for path_id in self.path_ids],
            dtype=np.float64,
        )

    def _stage_points(self, path_id, stage_index):
        path_scale, path_range = self.fallback_stages[stage_index]
        selected = []
        for x, y, z in self.paths[path_id]:
            scaled_x = path_scale * x
            scaled_y = path_scale * y
            if math.hypot(scaled_x, scaled_y) > path_range + 1.0e-9:
                break
            selected.append((scaled_x, scaled_y, path_scale * z))
        if not selected:
            selected.append((0.0, 0.0, 0.0))
        return selected

    def _build_stage_geometry(self):
        self.stage_end_x = []
        self.stage_end_y = []
        self.stage_end_yaw = []
        self.stage_sample_counts = []
        for stage_index, _ in enumerate(self.fallback_stages):
            end_x = []
            end_y = []
            end_yaw = []
            sample_counts = []
            for path_id in self.path_ids:
                path_id = int(path_id)
                points = self._stage_points(path_id, stage_index)
                count = len(points)
                end_x.append(points[-1][0])
                end_y.append(points[-1][1])
                sample_counts.append(max(1, count))
                if count >= len(self.paths[path_id]):
                    end_yaw.append(self.ends[path_id]["yaw"])
                else:
                    end_yaw.append(
                        self.ends[path_id]["vel"][2]
                        * count
                        * self.selected_prediction_step
                    )
            self.stage_end_x.append(np.asarray(end_x, dtype=np.float64))
            self.stage_end_y.append(np.asarray(end_y, dtype=np.float64))
            self.stage_end_yaw.append(np.asarray(end_yaw, dtype=np.float64))
            self.stage_sample_counts.append(
                np.asarray(sample_counts, dtype=np.float64)
            )

    def _load_correspondences(self):
        files = [
            "correspondences.ply",
            "correspondences_4.ply",
            "correspondences_5.ply",
            "correspondences_6.ply",
        ]
        for filename in files:
            path = os.path.join(self.path_folder, filename)
            if not os.path.exists(path):
                self.get_logger().warn("FIRST correspondence file missing: %s" % path)
                continue
            layer = {}
            with open(path, "r", encoding="utf-8") as handle:
                for line in handle:
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    voxel = int(parts[0])
                    path_ids = []
                    for token in parts[1:]:
                        path_id = int(token)
                        if path_id < 0:
                            break
                        path_ids.append(path_id)
                    if path_ids:
                        layer[voxel] = np.asarray(path_ids, dtype=np.int32)
            self.correspondence_layers.append(layer)

    def _on_odom(self, message):
        self.odom = message

    def _on_goal(self, message, source=""):
        self.goal = message
        self.goal_source = source
        frame = message.header.frame_id or "map"
        self.get_logger().info(
            "FIRST DWA goal received from %s frame=%s: x=%.2f y=%.2f"
            % (source, frame, message.pose.position.x, message.pose.position.y)
        )

    def _on_reset_navigation(self, message):
        if not message.data:
            return
        self.goal = None
        self.goal_source = ""
        self.selected_path_id = None
        self.dynamic_stop_latched = False
        self.dynamic_release_clear_frames = 0
        self.dynamic_release_last_generation = self.dynamic_cost_generation
        self.intrusion_latched = False
        self.intrusion_release_clear_frames = 0
        self._publish_zero()
        self._publish_empty_local_plan()
        self.get_logger().info("FIRST DWA state cleared for M20 recovery")

    def _on_cloud(self, message):
        self.local_scans_pub.publish(message)
        points = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if points.size == 0:
            self.obstacles = []
            self.obstacle_count = 0
            self.occupied_voxels = set()
            self.occupied_voxel_centers = np.empty((0, 2), dtype=np.float64)
            self.obstacle_grid = defaultdict(list)
            self._rebuild_path_penalties()
            self._publish_local_obstacle_grid()
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        distance = np.hypot(x, y)
        mask = (
            ~((np.abs(x) <= self.self_clear_x) & (np.abs(y) <= self.self_clear_y))
            & (distance >= 0.18)
            & (distance <= self.obstacle_range)
            & (z >= self.obstacle_height_threshold)
            & (z <= 1.2)
        )
        self.dynamic_masked_point_count = 0
        if (
            self.dynamic_obstacles.size > 0
            and self._stream_is_fresh(self.dynamic_obstacles_received_time)
        ):
            dynamic_hits = np.zeros(points.shape[0], dtype=bool)
            for center_x, center_y, radius in self.dynamic_obstacles:
                mask_radius = max(
                    0.0, radius + self.dynamic_point_mask_padding
                )
                dynamic_hits |= (
                    (x - center_x) * (x - center_x)
                    + (y - center_y) * (y - center_y)
                    <= mask_radius * mask_radius
                )
            self.dynamic_masked_point_count = int(
                np.count_nonzero(mask & dynamic_hits)
            )
            mask &= ~dynamic_hits
        filtered_x = x[mask]
        filtered_y = y[mask]
        self.obstacle_count = int(filtered_x.size)
        if self.use_correspondence_collision:
            self.obstacles = []
            self.obstacle_grid = defaultdict(list)
        else:
            self.obstacles = list(zip(filtered_x.tolist(), filtered_y.tolist()))
            self._rebuild_obstacle_grid()

        ix = np.floor(
            (filtered_x - self.grid_voxel_offset_x) / self.grid_voxel_size
        ).astype(np.int32)
        iy = np.floor(
            (filtered_y - self.grid_voxel_offset_y) / self.grid_voxel_size
        ).astype(np.int32)
        valid = (
            (ix >= 0)
            & (iy >= 0)
            & (ix < self.grid_voxel_num_x)
            & (iy < self.grid_voxel_num_y)
        )
        voxel_ids = ix[valid] * self.grid_voxel_num_y + iy[valid]
        unique_voxels, counts = np.unique(voxel_ids, return_counts=True)
        self.occupied_voxels = set(
            unique_voxels[counts >= self.grid_occupancy_threshold].tolist()
        )
        if self.occupied_voxels:
            occupied = np.asarray(sorted(self.occupied_voxels), dtype=np.int32)
            occupied_x = occupied // self.grid_voxel_num_y
            occupied_y = occupied % self.grid_voxel_num_y
            self.occupied_voxel_centers = np.column_stack(
                (
                    self.grid_voxel_offset_x
                    + (occupied_x.astype(np.float64) + 0.5) * self.grid_voxel_size,
                    self.grid_voxel_offset_y
                    + (occupied_y.astype(np.float64) + 0.5) * self.grid_voxel_size,
                )
            )
        else:
            self.occupied_voxel_centers = np.empty((0, 2), dtype=np.float64)
        self._rebuild_path_penalties()
        self._publish_local_obstacle_grid()

    def _publish_local_obstacle_grid(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "base_link"
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = self.grid_voxel_size
        grid.info.width = self.grid_voxel_num_x
        grid.info.height = self.grid_voxel_num_y
        grid.info.origin.position.x = self.grid_voxel_offset_x
        grid.info.origin.position.y = self.grid_voxel_offset_y
        grid.info.origin.orientation.w = 1.0
        data = [0] * (self.grid_voxel_num_x * self.grid_voxel_num_y)
        for voxel in self.occupied_voxels:
            ix = voxel // self.grid_voxel_num_y
            iy = voxel % self.grid_voxel_num_y
            if 0 <= ix < self.grid_voxel_num_x and 0 <= iy < self.grid_voxel_num_y:
                data[iy * self.grid_voxel_num_x + ix] = 100
        grid.data = data
        self.local_obstacle_grid_pub.publish(grid)

    def _rebuild_obstacle_grid(self):
        grid = defaultdict(list)
        resolution = self.grid_resolution
        for x, y in self.obstacles:
            cell = (int(math.floor(x / resolution)), int(math.floor(y / resolution)))
            grid[cell].append((x, y))
        self.obstacle_grid = grid

    def _grid_voxel_id(self, x, y):
        ix = int(math.floor((x - self.grid_voxel_offset_x) / self.grid_voxel_size))
        iy = int(math.floor((y - self.grid_voxel_offset_y) / self.grid_voxel_size))
        if ix < 0 or iy < 0 or ix >= self.grid_voxel_num_x or iy >= self.grid_voxel_num_y:
            return None
        return ix * self.grid_voxel_num_y + iy

    def _add_layer_hits(self, layer_index, target, occupied_voxels=None):
        if layer_index >= len(self.correspondence_layers):
            return
        layer = self.correspondence_layers[layer_index]
        voxels = self.occupied_voxels if occupied_voxels is None else occupied_voxels
        for voxel in voxels:
            path_ids = layer.get(voxel)
            if path_ids is not None:
                np.add.at(target, path_ids, 1)

    def _stage_occupied_voxels(self, path_scale, path_range):
        if self.occupied_voxel_centers.size == 0:
            return set()
        centers = self.occupied_voxel_centers
        distance = np.hypot(centers[:, 0], centers[:, 1])
        centers = centers[distance <= path_range + 1.0e-9]
        if centers.size == 0:
            return set()

        template_x = centers[:, 0] / path_scale
        template_y = centers[:, 1] / path_scale
        ix = np.floor(
            (template_x - self.grid_voxel_offset_x) / self.grid_voxel_size
        ).astype(np.int32)
        iy = np.floor(
            (template_y - self.grid_voxel_offset_y) / self.grid_voxel_size
        ).astype(np.int32)
        valid = (
            (ix >= 0)
            & (iy >= 0)
            & (ix < self.grid_voxel_num_x)
            & (iy < self.grid_voxel_num_y)
        )
        return set(
            np.unique(
                ix[valid] * self.grid_voxel_num_y + iy[valid]
            ).tolist()
        )

    def _rebuild_path_penalties(self):
        self.stage_blocked_paths = []
        self.stage_path_penalty1 = []
        self.stage_path_penalty2 = []
        self.stage_path_penalty3 = []
        for path_scale, path_range in self.fallback_stages:
            blocked = np.zeros(self.path_array_size, dtype=np.int32)
            penalty1 = np.zeros(self.path_array_size, dtype=np.int32)
            penalty2 = np.zeros(self.path_array_size, dtype=np.int32)
            penalty3 = np.zeros(self.path_array_size, dtype=np.int32)
            if self.use_correspondence_collision and self.correspondence_layers:
                voxels = self._stage_occupied_voxels(path_scale, path_range)
                self._add_layer_hits(0, blocked, voxels)
                self._add_layer_hits(1, penalty1, voxels)
                self._add_layer_hits(2, penalty2, voxels)
                self._add_layer_hits(3, penalty3, voxels)
            self.stage_blocked_paths.append(blocked)
            self.stage_path_penalty1.append(penalty1)
            self.stage_path_penalty2.append(penalty2)
            self.stage_path_penalty3.append(penalty3)

        self.blocked_paths = self.stage_blocked_paths[0]
        self.path_penalty1 = self.stage_path_penalty1[0]
        self.path_penalty2 = self.stage_path_penalty2[0]
        self.path_penalty3 = self.stage_path_penalty3[0]

    def _goal_in_base(self):
        if self.odom is None or self.goal is None:
            return None
        frame = (self.goal.header.frame_id or "map").lstrip("/")
        if frame in ("base_link", "base_footprint", "vehicle", "body", "m20/base_link"):
            local_x = self.goal.pose.position.x
            local_y = self.goal.pose.position.y
            distance = math.hypot(local_x, local_y)
            if distance > self.max_goal_range:
                scale = self.max_goal_range / distance
                local_x *= scale
                local_y *= scale
            return (
                local_x,
                local_y,
                yaw_from_quaternion(self.goal.pose.orientation),
                distance,
            )
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        dx = self.goal.pose.position.x - pose.position.x
        dy = self.goal.pose.position.y - pose.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = math.hypot(local_x, local_y)
        if distance > self.max_goal_range:
            scale = self.max_goal_range / distance
            local_x *= scale
            local_y *= scale
        goal_yaw = yaw_from_quaternion(self.goal.pose.orientation)
        local_yaw = normalize_angle(goal_yaw - yaw)
        return local_x, local_y, local_yaw, distance

    def _nearby_obstacles(self, x, y, radius):
        resolution = self.grid_resolution
        center = (int(math.floor(x / resolution)), int(math.floor(y / resolution)))
        cells = int(math.ceil(radius / resolution))
        for ix in range(center[0] - cells, center[0] + cells + 1):
            for iy in range(center[1] - cells, center[1] + cells + 1):
                for obstacle in self.obstacle_grid.get((ix, iy), []):
                    yield obstacle

    def _obstacle_cost(self, path_id):
        if self.use_correspondence_collision and self.correspondence_layers:
            if self.blocked_paths[path_id] >= self.point_per_path_threshold:
                return math.inf
            sample_count = max(1, len(self.paths.get(path_id, ())))
            return (
                self.weight_ob1 * self.path_penalty1[path_id]
                + self.weight_ob2 * self.path_penalty2[path_id]
                + self.weight_ob3 * self.path_penalty3[path_id]
            ) / sample_count

        cost = 0.0
        hard2 = self.hard_radius * self.hard_radius
        soft2 = self.soft_radius * self.soft_radius
        for x, y, _ in self.paths[path_id]:
            for ox, oy in self._nearby_obstacles(x, y, self.soft_radius):
                d2 = (x - ox) * (x - ox) + (y - oy) * (y - oy)
                if d2 <= hard2:
                    return math.inf
                if d2 <= soft2:
                    d = math.sqrt(max(d2, 1.0e-6))
                    cost += (self.soft_radius - d) / (self.soft_radius - self.hard_radius)
        return cost

    def _score_path(self, path_id, goal):
        goal_x, goal_y, goal_yaw, full_distance = goal
        end_x, end_y, _ = self.ends[path_id]["end"]
        end_yaw = self.ends[path_id]["yaw"]
        vx, vy, wz = self.ends[path_id]["vel"]
        vx = max(-self.max_speed_x, min(self.max_speed_x, vx))
        vy = max(-self.max_speed_y, min(self.max_speed_y, vy))
        wz = max(-self.max_theta, min(self.max_theta, wz))

        distance_cost = math.hypot(goal_x - end_x, goal_y - end_y)
        desired_heading = math.atan2(goal_y, goal_x)
        heading_cost = abs(normalize_angle(desired_heading - end_yaw))
        yaw_cost = 0.0
        if full_distance < 0.9:
            yaw_cost = abs(normalize_angle(goal_yaw - end_yaw))

        speed_bonus = math.hypot(vx, vy)
        cost = (
            self.goal_distance_weight * distance_cost
            + self.goal_heading_weight * heading_cost
            + 0.7 * yaw_cost
            - self.speed_weight * speed_bonus
            + self.reverse_penalty * max(0.0, -vx)
            + self.lateral_penalty * abs(vy)
        )
        obstacle = self._obstacle_cost(path_id)
        if not math.isfinite(obstacle):
            return math.inf
        cost += obstacle
        return cost

    def _select_stage_path(self, goal, stage_index, dynamic_fresh):
        goal_x, goal_y, goal_yaw, full_distance = goal
        path_scale, _ = self.fallback_stages[stage_index]
        vx = np.clip(
            path_scale * self.path_vel_x, -self.max_speed_x, self.max_speed_x
        )
        vy = np.clip(
            path_scale * self.path_vel_y, -self.max_speed_y, self.max_speed_y
        )
        wz = np.clip(self.path_vel_yaw, -self.max_theta, self.max_theta)
        distance_cost = np.hypot(
            goal_x - self.stage_end_x[stage_index],
            goal_y - self.stage_end_y[stage_index],
        )
        desired_heading = math.atan2(goal_y, goal_x)
        heading_delta = desired_heading - self.stage_end_yaw[stage_index]
        heading_cost = np.abs(np.arctan2(np.sin(heading_delta), np.cos(heading_delta)))
        if full_distance < 0.9:
            yaw_delta = goal_yaw - self.stage_end_yaw[stage_index]
            yaw_cost = np.abs(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))
        else:
            yaw_cost = 0.0
        score = (
            self.goal_distance_weight * distance_cost
            + self.goal_heading_weight * heading_cost
            + 0.7 * yaw_cost
            - self.speed_weight * np.hypot(vx, vy)
            + self.reverse_penalty * np.maximum(0.0, -vx)
            + self.lateral_penalty * np.abs(vy)
        )
        ids = self.path_ids
        score += (
            self.weight_ob1 * self.stage_path_penalty1[stage_index][ids]
            + self.weight_ob2 * self.stage_path_penalty2[stage_index][ids]
            + self.weight_ob3 * self.stage_path_penalty3[stage_index][ids]
        ) / self.stage_sample_counts[stage_index]
        static_blocked_mask = (
            self.stage_blocked_paths[stage_index][ids]
            >= self.point_per_path_threshold
        )
        blocked_mask = static_blocked_mask.copy()
        if dynamic_fresh:
            dynamic_cost = self.dynamic_costs[stage_index, ids]
            dynamic_blocked = dynamic_cost >= self.dynamic_block_cost
            self.dynamic_blocked_count = int(np.count_nonzero(dynamic_blocked))
            score += np.minimum(dynamic_cost, self.dynamic_block_cost)
            blocked_mask = np.logical_or(blocked_mask, dynamic_blocked)
        if (
            self.selected_path_id is not None
            and self.path_switch_penalty > 0.0
        ):
            score += self.path_switch_penalty * (ids != self.selected_path_id)
        score[blocked_mask] = np.inf
        blocked = int(np.count_nonzero(blocked_mask))
        best_index = int(np.argmin(score))
        if np.isfinite(score[best_index]):
            return int(ids[best_index]), float(score[best_index]), blocked
        return None, math.inf, blocked

    def _select_escape_path(self, goal, last_blocked):
        goal_x, goal_y, _, _ = goal
        ids = self.path_ids
        base_static_blocked = (
            self.stage_blocked_paths[0][ids] >= self.point_per_path_threshold
        )
        base_vx = np.clip(self.path_vel_x, -self.max_speed_x, self.max_speed_x)
        base_vy = np.clip(self.path_vel_y, -self.max_speed_y, self.max_speed_y)
        base_distance_cost = np.hypot(
            goal_x - self.stage_end_x[0], goal_y - self.stage_end_y[0]
        )
        desired_heading = math.atan2(goal_y, goal_x)
        heading_delta = desired_heading - self.stage_end_yaw[0]
        base_heading_cost = np.abs(
            np.arctan2(np.sin(heading_delta), np.cos(heading_delta))
        )
        escape_fresh = (
            self.dynamic_cost_active
            and self.dynamic_escape_costs is not None
            and self._stream_is_fresh(self.dynamic_escape_received_time)
        )
        if escape_fresh:
            translation_speed = np.hypot(base_vx, base_vy)
            escape_candidate = (
                ~base_static_blocked
                & (translation_speed >= self.escape_min_translation_speed)
                & (translation_speed <= self.escape_max_translation_speed)
            )
            if np.any(escape_candidate):
                escape_score = (
                    self.dynamic_escape_costs[ids]
                    + self.escape_goal_cost_blend
                    * (
                        self.goal_distance_weight * base_distance_cost
                        + self.goal_heading_weight * base_heading_cost
                    )
                )
                escape_score[~escape_candidate] = np.inf
                escape_index = int(np.argmin(escape_score))
                if np.isfinite(escape_score[escape_index]):
                    self.selection_mode = "escape"
                    self.selected_stage_index = 0
                    self.selected_scale = self.fallback_stages[0][0]
                    self.selected_range = self.fallback_stages[0][1]
                    return (
                        int(ids[escape_index]),
                        float(escape_score[escape_index]),
                        last_blocked,
                        0,
                    )
        return None

    def _select_path(self, goal):
        self.dynamic_cost_active = False
        self.dynamic_blocked_count = 0
        self.dynamic_cost_age = math.inf
        if (
            self.dynamic_costs is not None
            and self.dynamic_cost_received_time is not None
        ):
            self.dynamic_cost_age = (
                self.get_clock().now() - self.dynamic_cost_received_time
            ).nanoseconds * 1.0e-9
            self.dynamic_cost_active = (
                self.dynamic_cost_age <= self.dynamic_cost_timeout
            )

        self.stage_blocked_counts = []
        if self.require_dynamic_input and not self.dynamic_cost_active:
            self.selection_mode = "dynamic_unavailable"
            self.selected_stage_index = 0
            self.selected_scale, self.selected_range = self.fallback_stages[0]
            self.stage_blocked_counts = [
                int(
                    np.count_nonzero(
                        blocked >= self.point_per_path_threshold
                    )
                )
                for blocked in self.stage_blocked_paths
            ]
            return None, math.inf, len(self.path_ids), None

        last_blocked = len(self.path_ids)
        for stage_index, (path_scale, path_range) in enumerate(
            self.fallback_stages
        ):
            path_id, score, blocked = self._select_stage_path(
                goal, stage_index, self.dynamic_cost_active
            )
            self.stage_blocked_counts.append(blocked)
            last_blocked = blocked
            if path_id is None:
                continue

            self.selected_stage_index = stage_index
            self.selected_scale = path_scale
            self.selected_range = path_range
            if stage_index == 0:
                self.selection_mode = "normal"
            elif path_scale < self.fallback_stages[stage_index - 1][0]:
                self.selection_mode = "path_scale"
            else:
                self.selection_mode = "path_range"
            return path_id, score, blocked, stage_index

        self.selection_mode = "blocked"
        self.selected_stage_index = len(self.fallback_stages) - 1
        self.selected_scale, self.selected_range = self.fallback_stages[-1]
        escape = self._select_escape_path(goal, last_blocked)
        if escape is not None:
            return escape
        if self._stop_intrusion_is_fresh():
            self.selection_mode = "intrusion_blocked"
        return None, math.inf, last_blocked, None

    def _publish_local_plan(self, path_id, stage_index):
        if self.odom is None or path_id is None:
            return
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        for x, y, z in self._stage_points(path_id, stage_index):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)
        self.track_path_pub.publish(message)
        self._publish_selected_prediction(path_id, stage_index)

    def _candidate_state_at(self, path_id, candidate_time, stage_index):
        points = self._stage_points(path_id, stage_index)
        end = self.ends[path_id]
        path_scale, _ = self.fallback_stages[stage_index]
        step = self.selected_prediction_step
        exact_horizon = len(points) * step
        vx, vy, yaw_rate = end["vel"]
        vx *= path_scale
        vy *= path_scale
        if candidate_time <= exact_horizon + 1.0e-9:
            index = min(
                max(0, int(math.ceil(candidate_time / step) - 1)),
                len(points) - 1,
            )
            x, y, _ = points[index]
            return x, y, yaw_rate * candidate_time

        extension_time = candidate_time - exact_horizon
        if len(points) >= len(self.paths[path_id]):
            yaw_start = end["yaw"]
        else:
            yaw_start = yaw_rate * exact_horizon
        yaw_end = yaw_start + yaw_rate * extension_time
        if abs(yaw_rate) > 1.0e-6:
            delta_x = (
                vx * (math.sin(yaw_end) - math.sin(yaw_start))
                + vy * (math.cos(yaw_end) - math.cos(yaw_start))
            ) / yaw_rate
            delta_y = (
                vx * (math.cos(yaw_start) - math.cos(yaw_end))
                + vy * (math.sin(yaw_end) - math.sin(yaw_start))
            ) / yaw_rate
        else:
            cosine = math.cos(yaw_start)
            sine = math.sin(yaw_start)
            delta_x = (
                cosine * vx - sine * vy
            ) * extension_time
            delta_y = (
                sine * vx + cosine * vy
            ) * extension_time
        return points[-1][0] + delta_x, points[-1][1] + delta_y, yaw_end

    def _publish_selected_prediction(self, path_id, stage_index):
        now = self.get_clock().now().to_msg()
        header_frame = "base_link"
        markers = MarkerArray()

        clear = Marker()
        clear.header.stamp = now
        clear.header.frame_id = header_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        status = Marker()
        status.header = clear.header
        status.ns = "first_v2_stage"
        status.id = 100
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = 0.0
        status.pose.position.y = 0.0
        status.pose.position.z = 0.85
        status.pose.orientation.w = 1.0
        status.scale.z = 0.24
        status.color.r = 0.10
        status.color.g = 0.20 if self.selection_mode == "escape" else 0.75
        status.color.b = 1.0 if self.selection_mode != "escape" else 0.15
        status.color.a = 1.0
        status.text = "V2 %s  scale %.2f  range %.2f" % (
            self.selection_mode,
            self.selected_scale,
            self.selected_range,
        )
        status.lifetime.nanosec = 300_000_000
        markers.markers.append(status)

        exact = Marker()
        exact.header = clear.header
        exact.ns = "first_v2_official"
        exact.id = 0
        exact.type = Marker.LINE_STRIP
        exact.action = Marker.ADD
        exact.pose.orientation.w = 1.0
        exact.scale.x = 0.09
        exact.color.r = 0.05
        exact.color.g = 0.45
        exact.color.b = 1.0
        exact.color.a = 1.0
        exact.lifetime.nanosec = 300_000_000
        stage_points = self._stage_points(path_id, stage_index)
        for x, y, z in stage_points:
            point = Point()
            point.x = x
            point.y = y
            point.z = max(0.08, z)
            exact.points.append(point)
        markers.markers.append(exact)

        extension = Marker()
        extension.header = clear.header
        extension.ns = "first_v2_integrated"
        extension.id = 1
        extension.type = Marker.LINE_LIST
        extension.action = Marker.ADD
        extension.pose.orientation.w = 1.0
        extension.scale.x = 0.07
        extension.color.r = 0.80
        extension.color.g = 0.10
        extension.color.b = 0.95
        extension.color.a = 0.95
        extension.lifetime.nanosec = 300_000_000

        exact_horizon = (
            len(stage_points) * self.selected_prediction_step
        )
        previous = self._candidate_state_at(path_id, exact_horizon, stage_index)
        sample_count = int(
            math.floor(
                (self.selected_prediction_horizon - exact_horizon)
                / self.selected_prediction_step
                + 1.0e-6
            )
        )
        for index in range(sample_count):
            prediction_time = (
                exact_horizon + (index + 1) * self.selected_prediction_step
            )
            current = self._candidate_state_at(
                path_id, prediction_time, stage_index
            )
            if index % 2 == 0:
                for state in (previous, current):
                    point = Point()
                    point.x = state[0]
                    point.y = state[1]
                    point.z = 0.10
                    extension.points.append(point)
            previous = current
        markers.markers.append(extension)

        for marker_id, prediction_time in enumerate((1.0, 2.0, 3.0), start=10):
            x, y, _ = self._candidate_state_at(
                path_id, prediction_time, stage_index
            )
            label = Marker()
            label.header = clear.header
            label.ns = "first_prediction_time"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 0.15 if prediction_time <= 1.0 else 0.75
            label.color.g = 0.35
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = "V2 t=%.0fs" % prediction_time
            label.lifetime.nanosec = 300_000_000
            markers.markers.append(label)

        self.selected_prediction_markers_pub.publish(markers)

    def _publish_stage(self, reason):
        message = String()
        message.data = (
            "mode=%s stage=%s scale=%.2f range=%.2f reason=%s"
            % (
                self.selection_mode,
                str(self.selected_stage_index),
                self.selected_scale,
                self.selected_range,
                reason,
            )
        )
        self.planner_stage_pub.publish(message)

    def _publish_zero(self):
        self._publish_command(0.0, 0.0, 0.0)

    def _publish_empty_local_plan(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        self.path_pub.publish(message)
        self.track_path_pub.publish(message)
        clear_markers = MarkerArray()
        clear = Marker()
        clear.header = message.header
        clear.action = Marker.DELETEALL
        clear_markers.markers.append(clear)
        self.selected_prediction_markers_pub.publish(clear_markers)

    def _publish_stop_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
        self.path_pub.publish(message)
        self.track_path_pub.publish(message)
        clear_markers = MarkerArray()
        clear = Marker()
        clear.header = message.header
        clear.action = Marker.DELETEALL
        clear_markers.markers.append(clear)
        self.selected_prediction_markers_pub.publish(clear_markers)

    def _publish_command(self, vx, vy, wz):
        if (
            self.true_robot_speed_limits is not None
            and self._stream_is_fresh(self.true_robot_speed_limits_received_time)
        ):
            vx = float(
                np.clip(
                    vx,
                    -self.true_robot_speed_limits[0],
                    self.true_robot_speed_limits[0],
                )
            )
            vy = float(
                np.clip(
                    vy,
                    -self.true_robot_speed_limits[1],
                    self.true_robot_speed_limits[1],
                )
            )
            wz = float(
                np.clip(
                    wz,
                    -self.true_robot_speed_limits[2],
                    self.true_robot_speed_limits[2],
                )
            )
        command = NavCmd()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 0
        command.data.x_vel = float(vx)
        command.data.y_vel = float(vy)
        command.data.yaw_vel = float(wz)
        self.cmd_pub.publish(command)

    def _control_update(self):
        goal = self._goal_in_base()
        if goal is None:
            if self.publish_zero_without_goal:
                self._publish_zero()
                self.selection_mode = "idle"
                self.dynamic_stop_latched = False
                self.dynamic_release_clear_frames = 0
                self.dynamic_release_last_generation = (
                    self.dynamic_cost_generation
                )
                self.intrusion_latched = False
                self.intrusion_release_clear_frames = 0
                self._publish_stage("no_goal")
            return

        goal_x, goal_y, goal_yaw, full_distance = goal
        yaw_ok = (not self.require_goal_yaw) or abs(goal_yaw) < self.yaw_tolerance
        if full_distance < self.goal_tolerance and yaw_ok:
            self._publish_zero()
            self._publish_empty_local_plan()
            self.selected_path_id = None
            self.goal = None
            self.goal_source = ""
            self.selection_mode = "goal_reached"
            self.dynamic_stop_latched = False
            self.dynamic_release_clear_frames = 0
            self.dynamic_release_last_generation = self.dynamic_cost_generation
            self.intrusion_latched = False
            self.intrusion_release_clear_frames = 0
            self._publish_stage("goal_reached")
            return

        path_id, best_score, blocked, stage_index = self._select_path(goal)
        if path_id is None:
            if self._dynamic_caused_no_solution():
                self.dynamic_stop_latched = True
                self.dynamic_release_last_generation = (
                    self.dynamic_cost_generation
                )
            self.dynamic_release_clear_frames = 0
            reason = (
                "dynamic_input_unavailable"
                if self.selection_mode == "dynamic_unavailable"
                else (
                    "intrusion_no_escape"
                    if self.selection_mode == "intrusion_blocked"
                    else "all_paths_blocked"
                )
            )
            self._log_diagnostic(goal, None, math.inf, blocked, reason)
            self._publish_zero()
            self._publish_stop_path()
            self._publish_stage(reason)
            return

        intrusion_requires_motion = self._stop_intrusion_is_fresh()
        if self.dynamic_stop_latched and not intrusion_requires_motion:
            if (
                self.dynamic_cost_generation
                != self.dynamic_release_last_generation
            ):
                self.dynamic_release_last_generation = (
                    self.dynamic_cost_generation
                )
                self.dynamic_release_clear_frames += 1
            if (
                self.dynamic_release_clear_frames
                < self.dynamic_release_confirmation_frames
            ):
                self.selection_mode = "dynamic_release_wait"
                self._publish_zero()
                self.selected_path_id = path_id
                self._publish_local_plan(path_id, stage_index)
                self._publish_stage(
                    "safe_frames_%d_of_%d"
                    % (
                        self.dynamic_release_clear_frames,
                        self.dynamic_release_confirmation_frames,
                    )
                )
                self._log_diagnostic(
                    goal, path_id, best_score, blocked, "dynamic_release_wait"
                )
                return
            self.dynamic_stop_latched = False
            self.dynamic_release_clear_frames = 0
        elif intrusion_requires_motion:
            self.dynamic_stop_latched = False
            self.dynamic_release_clear_frames = 0

        vx, vy, wz = self.ends[path_id]["vel"]
        if self.selection_mode != "escape":
            vx *= self.selected_scale
            vy *= self.selected_scale
        vx = max(-self.max_speed_x, min(self.max_speed_x, vx))
        vy = max(-self.max_speed_y, min(self.max_speed_y, vy))
        wz = max(-self.max_theta, min(self.max_theta, wz))
        self._publish_command(vx, vy, wz)
        self.selected_path_id = path_id
        self._publish_local_plan(path_id, stage_index)
        reason = (
            "intrusion_escape"
            if self.selection_mode == "escape" and intrusion_requires_motion
            else ("dynamic_escape" if self.selection_mode == "escape" else "selected")
        )
        self._publish_stage(reason)
        self._log_diagnostic(goal, path_id, best_score, blocked, reason)

    def _log_diagnostic(self, goal, path_id, score, blocked, reason):
        now = self.get_clock().now()
        if self.last_diagnostic_time is not None:
            if (now - self.last_diagnostic_time).nanoseconds * 1.0e-9 < 1.0:
                return
        self.last_diagnostic_time = now
        goal_x, goal_y, _, full_distance = goal
        self.get_logger().info(
            "FIRST V2 %s goal_base=(%.2f, %.2f) dist=%.2f obstacles=%d "
            "voxels=%d blocked=%d/%d dynamic=%s dynamic_blocked=%d "
            "dynamic_age=%.3f dynamic_masked_points=%d mode=%s "
            "stage=%d scale=%.2f range=%.2f stage_blocked=%s "
            "path=%s score=%.3f"
            % (
                reason,
                goal_x,
                goal_y,
                full_distance,
                self.obstacle_count,
                len(self.occupied_voxels),
                blocked,
                len(self.ends),
                "active" if self.dynamic_cost_active else "stale",
                self.dynamic_blocked_count,
                self.dynamic_cost_age
                if math.isfinite(self.dynamic_cost_age)
                else -1.0,
                self.dynamic_masked_point_count,
                self.selection_mode,
                self.selected_stage_index,
                self.selected_scale,
                self.selected_range,
                str(self.stage_blocked_counts),
                str(path_id),
                score if math.isfinite(score) else -1.0,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = FirstPlannerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
