#!/usr/bin/env python3

import csv
import json
import math
import os
import re
import time
from datetime import datetime

import rclpy
from drdds.msg import NavCmd
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


STAGE_PATTERN = re.compile(r"(\w+)=([^ ]+)")


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def finite_or_none(value):
    return float(value) if math.isfinite(value) else None


class GoalDiagnostics(Node):
    TIMELINE_FIELDS = [
        "sim_time",
        "elapsed",
        "pose_x",
        "pose_y",
        "pose_yaw",
        "odom_vx",
        "odom_vy",
        "odom_wz",
        "odom_speed",
        "cmd_vx",
        "cmd_vy",
        "cmd_wz",
        "cmd_speed",
        "goal_x",
        "goal_y",
        "goal_distance",
        "local_goal_x",
        "local_goal_y",
        "local_goal_distance",
        "planner_mode",
        "planner_stage",
        "path_scale",
        "path_range",
        "planner_reason",
        "planner_state_age",
        "dynamic_cost_age",
        "dynamic_blocked_by_stage",
        "dynamic_min_cost_by_stage",
        "escape_safe_count",
        "escape_min_cost",
        "dynamic_object_count",
        "dynamic_min_clearance",
        "prediction_marker_count",
        "prediction_labels",
        "local_grid_occupied",
        "global_path_poses",
        "local_path_poses",
        "local_path_end_x",
        "local_path_end_y",
        "stop_candidate",
        "stop_cause",
    ]

    EVENT_FIELDS = [
        "sim_time",
        "elapsed",
        "mode",
        "stage",
        "scale",
        "range",
        "reason",
        "dynamic_blocked_by_stage",
        "escape_safe_count",
        "dynamic_object_count",
    ]

    def __init__(self):
        super().__init__("m20_v2_goal_diagnostics")
        self.declare_parameter(
            "output_directory",
            "/home/ubuntu/social_nav_ws/logs/v2_goal_diagnostics",
        )
        self.declare_parameter("sample_frequency", 10.0)
        self.declare_parameter("session_timeout", 300.0)
        self.declare_parameter("stationary_speed_threshold", 0.05)
        self.declare_parameter("command_speed_threshold", 0.08)
        self.declare_parameter("stationary_confirm_duration", 0.80)
        self.declare_parameter("goal_active_distance", 0.50)
        self.declare_parameter("stream_stale_timeout", 0.50)
        self.declare_parameter("dynamic_block_cost", 1000000.0)
        self.declare_parameter("candidate_count", 4641)
        self.declare_parameter("fallback_stage_count", 6)

        self.output_directory = os.path.expanduser(
            str(self.get_parameter("output_directory").value)
        )
        os.makedirs(self.output_directory, exist_ok=True)

        self.odom = None
        self.odom_received = None
        self.command = None
        self.command_received = None
        self.goal = None
        self.local_goal = None
        self.global_path = None
        self.local_path = None
        self.planner_state = {
            "mode": "unknown",
            "stage": "-1",
            "scale": "0.0",
            "range": "0.0",
            "reason": "missing",
        }
        self.planner_state_received = None
        self.dynamic_cost_received = None
        self.dynamic_blocked_by_stage = []
        self.dynamic_min_cost_by_stage = []
        self.escape_received = None
        self.escape_safe_count = 0
        self.escape_min_cost = math.inf
        self.dynamic_objects = []
        self.dynamic_objects_received = None
        self.prediction_marker_count = 0
        self.prediction_labels = []
        self.local_grid_occupied = 0

        self.session = None
        self.last_goal_signature = None
        self.last_goal_wall_time = 0.0
        self.stationary_since = None
        self.active_stop_episode = None
        self.stop_episodes = []
        self.timeline_handle = None
        self.timeline_writer = None
        self.events_handle = None
        self.events_writer = None
        self.objects_handle = None

        self.status_publisher = self.create_publisher(
            String, "/m20/v2/goal_diagnostic_status", 10
        )
        for topic in ("/goal_pose", "/target_goal", "/move_base_simple/goal"):
            self.create_subscription(
                PoseStamped,
                topic,
                lambda message, source=topic: self._on_global_goal(
                    message, source
                ),
                10,
            )
        self.create_subscription(
            Odometry, "/m20/ground_truth", self._on_odometry, 20
        )
        self.create_subscription(NavCmd, "/NAV_CMD", self._on_command, 20)
        self.create_subscription(
            PoseStamped, "/local_goal", self._on_local_goal, 20
        )
        self.create_subscription(Path, "/path_Astar", self._on_global_path, 10)
        self.create_subscription(
            Path, "/m20/v2/local_plan", self._on_local_path, 10
        )
        self.create_subscription(
            String, "/m20/v2/planner_stage", self._on_planner_stage, 20
        )
        self.create_subscription(
            Float32MultiArray,
            "/m20/v2/first_dynamic_costs",
            self._on_dynamic_costs,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/m20/v2/first_dynamic_escape_costs",
            self._on_escape_costs,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/m20/v2/first_dynamic_obstacles_local",
            self._on_dynamic_objects,
            10,
        )
        self.create_subscription(
            MarkerArray,
            "/m20/v2/dynamic_prediction_markers",
            self._on_prediction_markers,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/m20/v2/local_obstacle_grid",
            self._on_local_grid,
            10,
        )
        self.create_subscription(
            Bool, "/m20/reset_navigation", self._on_reset, 10
        )

        frequency = float(self.get_parameter("sample_frequency").value)
        self.create_timer(1.0 / max(1.0, frequency), self._sample)
        self.get_logger().info(
            "V2 goal diagnostics armed: waiting for a user goal; output=%s"
            % self.output_directory
        )

    def _sim_time(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _age(self, received_time):
        if received_time is None:
            return math.inf
        return max(0.0, self._sim_time() - received_time)

    def _elapsed(self):
        if self.session is None:
            return 0.0
        return max(0.0, self._sim_time() - self.session["start_sim_time"])

    def _goal_signature(self, message):
        return (
            round(float(message.pose.position.x), 3),
            round(float(message.pose.position.y), 3),
            round(yaw_from_quaternion(message.pose.orientation), 3),
        )

    def _on_global_goal(self, message, source):
        signature = self._goal_signature(message)
        wall_now = time.monotonic()
        if (
            signature == self.last_goal_signature
            and wall_now - self.last_goal_wall_time < 1.0
        ):
            return
        self.last_goal_signature = signature
        self.last_goal_wall_time = wall_now
        if self.session is not None:
            self._finish_session("superseded_by_new_goal")
        self.goal = message
        self._start_session(message, source)

    def _on_odometry(self, message):
        self.odom = message
        self.odom_received = self._sim_time()

    def _on_command(self, message):
        self.command = message
        self.command_received = self._sim_time()

    def _on_local_goal(self, message):
        self.local_goal = message

    def _on_global_path(self, message):
        self.global_path = message

    def _on_local_path(self, message):
        self.local_path = message

    def _on_planner_stage(self, message):
        parsed = dict(STAGE_PATTERN.findall(message.data))
        previous = dict(self.planner_state)
        self.planner_state.update(parsed)
        self.planner_state_received = self._sim_time()
        if self.session is not None and self.planner_state != previous:
            self._write_planner_event()
        if (
            self.session is not None
            and self.planner_state.get("mode") == "goal_reached"
        ):
            self._sample()
            self._finish_session("goal_reached")

    def _on_dynamic_costs(self, message):
        stage_count = int(self.get_parameter("fallback_stage_count").value)
        candidate_count = int(self.get_parameter("candidate_count").value)
        expected = stage_count * candidate_count
        if len(message.data) != expected:
            self.dynamic_blocked_by_stage = []
            self.dynamic_min_cost_by_stage = []
            return
        threshold = float(self.get_parameter("dynamic_block_cost").value)
        blocked = []
        minimums = []
        for stage in range(stage_count):
            begin = stage * candidate_count
            values = message.data[begin : begin + candidate_count]
            blocked.append(sum(value >= threshold for value in values))
            finite_values = [
                float(value)
                for value in values
                if math.isfinite(value) and value < threshold
            ]
            minimums.append(min(finite_values) if finite_values else math.inf)
        self.dynamic_blocked_by_stage = blocked
        self.dynamic_min_cost_by_stage = minimums
        self.dynamic_cost_received = self._sim_time()

    def _on_escape_costs(self, message):
        threshold = float(self.get_parameter("dynamic_block_cost").value)
        safe = [
            float(value)
            for value in message.data
            if math.isfinite(value) and value < threshold
        ]
        self.escape_safe_count = len(safe)
        self.escape_min_cost = min(safe) if safe else math.inf
        self.escape_received = self._sim_time()

    def _on_dynamic_objects(self, message):
        values = list(message.data)
        self.dynamic_objects = [
            {
                "x": float(values[index]),
                "y": float(values[index + 1]),
                "radius": float(values[index + 2]),
            }
            for index in range(0, len(values) - 2, 3)
        ]
        self.dynamic_objects_received = self._sim_time()
        if self.session is not None and self.objects_handle is not None:
            record = {
                "sim_time": self._sim_time(),
                "elapsed": self._elapsed(),
                "objects": self.dynamic_objects,
            }
            self.objects_handle.write(
                json.dumps(
                    record, ensure_ascii=True, separators=(",", ":")
                )
                + "\n"
            )

    def _on_prediction_markers(self, message):
        active = [
            marker
            for marker in message.markers
            if marker.action == Marker.ADD
        ]
        self.prediction_marker_count = len(active)
        self.prediction_labels = [
            marker.text
            for marker in active
            if marker.type == Marker.TEXT_VIEW_FACING and marker.text
        ]

    def _on_local_grid(self, message):
        self.local_grid_occupied = sum(value > 0 for value in message.data)

    def _on_reset(self, message):
        if message.data and self.session is not None:
            self._finish_session("navigation_reset")

    def _start_session(self, goal, source):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        session_directory = os.path.join(
            self.output_directory, "goal_%s" % stamp
        )
        os.makedirs(session_directory, exist_ok=False)
        self.session = {
            "directory": session_directory,
            "source": source,
            "start_sim_time": self._sim_time(),
            "start_wall_time": datetime.now().isoformat(
                timespec="milliseconds"
            ),
            "goal": {
                "frame": goal.header.frame_id or "map",
                "x": float(goal.pose.position.x),
                "y": float(goal.pose.position.y),
                "yaw": yaw_from_quaternion(goal.pose.orientation),
            },
        }
        self.stop_episodes = []
        self.stationary_since = None
        self.active_stop_episode = None

        self.timeline_handle = open(
            os.path.join(session_directory, "timeline.csv"),
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        self.timeline_writer = csv.DictWriter(
            self.timeline_handle, fieldnames=self.TIMELINE_FIELDS
        )
        self.timeline_writer.writeheader()
        self.events_handle = open(
            os.path.join(session_directory, "planner_events.csv"),
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        self.events_writer = csv.DictWriter(
            self.events_handle, fieldnames=self.EVENT_FIELDS
        )
        self.events_writer.writeheader()
        self.objects_handle = open(
            os.path.join(session_directory, "dynamic_objects.jsonl"),
            "w",
            encoding="utf-8",
            buffering=1,
        )
        with open(
            os.path.join(session_directory, "session.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(self.session, handle, indent=2, ensure_ascii=True)
        self._write_planner_event()
        self.get_logger().info(
            "V2 goal diagnostic session started: %s" % session_directory
        )
        self.status_publisher.publish(
            String(data="recording directory=%s" % session_directory)
        )

    def _pose_and_motion(self):
        if self.odom is None:
            return {
                key: math.nan
                for key in ("x", "y", "yaw", "vx", "vy", "wz", "speed")
            }
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw_from_quaternion(pose.orientation),
            "vx": float(twist.linear.x),
            "vy": float(twist.linear.y),
            "wz": float(twist.angular.z),
            "speed": math.hypot(twist.linear.x, twist.linear.y),
        }

    def _command_motion(self):
        if self.command is None:
            return {
                key: math.nan for key in ("vx", "vy", "wz", "speed")
            }
        data = self.command.data
        return {
            "vx": float(data.x_vel),
            "vy": float(data.y_vel),
            "wz": float(data.yaw_vel),
            "speed": math.hypot(data.x_vel, data.y_vel),
        }

    def _distance_to(self, target, motion):
        if target is None or not math.isfinite(motion["x"]):
            return math.inf
        return math.hypot(
            target.pose.position.x - motion["x"],
            target.pose.position.y - motion["y"],
        )

    def _dynamic_min_clearance(self):
        if not self.dynamic_objects:
            return math.inf
        return min(
            math.hypot(item["x"], item["y"]) - item["radius"]
            for item in self.dynamic_objects
        )

    def _stop_cause(self, motion, command):
        stale_timeout = float(self.get_parameter("stream_stale_timeout").value)
        mode = self.planner_state.get("mode", "unknown")
        reason = self.planner_state.get("reason", "missing")
        if self._age(self.odom_received) > stale_timeout:
            return "odometry_stale"
        if self._age(self.planner_state_received) > stale_timeout:
            return "planner_state_stale"
        if self._age(self.dynamic_cost_received) > stale_timeout:
            return "dynamic_cost_stale"
        if (
            mode == "dynamic_unavailable"
            or reason == "dynamic_input_unavailable"
        ):
            return "dynamic_input_unavailable"
        if mode == "blocked" or reason == "all_paths_blocked":
            candidate_count = int(
                self.get_parameter("candidate_count").value
            )
            if (
                self.dynamic_blocked_by_stage
                and min(self.dynamic_blocked_by_stage) >= candidate_count
            ):
                return "dynamic_blocked_all_candidates"
            if self.escape_safe_count == 0:
                return "all_paths_blocked_no_safe_escape"
            return "blocked_with_reported_safe_escape"
        if self.local_goal is None or self.global_path is None:
            return "global_or_local_goal_missing"
        if len(self.global_path.poses) == 0:
            return "global_path_empty"
        command_threshold = float(
            self.get_parameter("command_speed_threshold").value
        )
        commanded = (
            command["speed"] >= command_threshold
            or abs(command["wz"]) >= command_threshold
        )
        if not commanded:
            if mode in ("normal", "path_scale", "path_range", "escape"):
                return "selected_near_zero_command"
            return "zero_command"
        if motion["speed"] < float(
            self.get_parameter("stationary_speed_threshold").value
        ):
            return "controller_not_following_nonzero_command"
        return "moving"

    def _update_stop_episode(self, stop_candidate, cause):
        now = self._sim_time()
        confirm = float(
            self.get_parameter("stationary_confirm_duration").value
        )
        if not stop_candidate:
            self.stationary_since = None
            self._close_stop_episode(now)
            return
        if self.stationary_since is None:
            self.stationary_since = now
            return
        if now - self.stationary_since < confirm:
            return
        if self.active_stop_episode is None:
            self.active_stop_episode = {
                "start": self.stationary_since,
                "start_elapsed": max(
                    0.0,
                    self.stationary_since - self.session["start_sim_time"],
                ),
                "cause": cause,
            }
            self.get_logger().warn(
                "V2 sustained stop detected: cause=%s session=%s"
                % (cause, self.session["directory"])
            )
            self.status_publisher.publish(
                String(data="sustained_stop cause=%s" % cause)
            )
        elif self.active_stop_episode["cause"] != cause:
            self._close_stop_episode(now)
            self.active_stop_episode = {
                "start": now,
                "start_elapsed": self._elapsed(),
                "cause": cause,
            }

    def _close_stop_episode(self, now):
        if self.active_stop_episode is None:
            return
        episode = dict(self.active_stop_episode)
        episode["end"] = now
        episode["end_elapsed"] = max(
            0.0, now - self.session["start_sim_time"]
        )
        episode["duration"] = max(0.0, now - episode["start"])
        self.stop_episodes.append(episode)
        self.active_stop_episode = None

    def _write_planner_event(self):
        if self.session is None or self.events_writer is None:
            return
        self.events_writer.writerow(
            {
                "sim_time": "%.3f" % self._sim_time(),
                "elapsed": "%.3f" % self._elapsed(),
                "mode": self.planner_state.get("mode", "unknown"),
                "stage": self.planner_state.get("stage", "-1"),
                "scale": self.planner_state.get("scale", "0.0"),
                "range": self.planner_state.get("range", "0.0"),
                "reason": self.planner_state.get("reason", "missing"),
                "dynamic_blocked_by_stage": json.dumps(
                    self.dynamic_blocked_by_stage, separators=(",", ":")
                ),
                "escape_safe_count": self.escape_safe_count,
                "dynamic_object_count": len(self.dynamic_objects),
            }
        )

    def _sample(self):
        if self.session is None:
            return
        if self._elapsed() > float(
            self.get_parameter("session_timeout").value
        ):
            self._finish_session("timeout")
            return

        motion = self._pose_and_motion()
        command = self._command_motion()
        goal_distance = self._distance_to(self.goal, motion)
        local_goal_distance = self._distance_to(self.local_goal, motion)
        active_distance = float(
            self.get_parameter("goal_active_distance").value
        )
        stop_candidate = (
            goal_distance > active_distance
            and motion["speed"]
            < float(self.get_parameter("stationary_speed_threshold").value)
        )
        cause = self._stop_cause(motion, command)
        self._update_stop_episode(stop_candidate, cause)

        local_end_x = math.nan
        local_end_y = math.nan
        local_path_count = 0
        if self.local_path is not None:
            local_path_count = len(self.local_path.poses)
            if self.local_path.poses:
                local_end = self.local_path.poses[-1].pose.position
                local_end_x = float(local_end.x)
                local_end_y = float(local_end.y)

        row = {
            "sim_time": "%.3f" % self._sim_time(),
            "elapsed": "%.3f" % self._elapsed(),
            "pose_x": motion["x"],
            "pose_y": motion["y"],
            "pose_yaw": motion["yaw"],
            "odom_vx": motion["vx"],
            "odom_vy": motion["vy"],
            "odom_wz": motion["wz"],
            "odom_speed": motion["speed"],
            "cmd_vx": command["vx"],
            "cmd_vy": command["vy"],
            "cmd_wz": command["wz"],
            "cmd_speed": command["speed"],
            "goal_x": self.goal.pose.position.x if self.goal else math.nan,
            "goal_y": self.goal.pose.position.y if self.goal else math.nan,
            "goal_distance": finite_or_none(goal_distance),
            "local_goal_x": (
                self.local_goal.pose.position.x
                if self.local_goal
                else math.nan
            ),
            "local_goal_y": (
                self.local_goal.pose.position.y
                if self.local_goal
                else math.nan
            ),
            "local_goal_distance": finite_or_none(local_goal_distance),
            "planner_mode": self.planner_state.get("mode", "unknown"),
            "planner_stage": self.planner_state.get("stage", "-1"),
            "path_scale": self.planner_state.get("scale", "0.0"),
            "path_range": self.planner_state.get("range", "0.0"),
            "planner_reason": self.planner_state.get("reason", "missing"),
            "planner_state_age": finite_or_none(
                self._age(self.planner_state_received)
            ),
            "dynamic_cost_age": finite_or_none(
                self._age(self.dynamic_cost_received)
            ),
            "dynamic_blocked_by_stage": json.dumps(
                self.dynamic_blocked_by_stage, separators=(",", ":")
            ),
            "dynamic_min_cost_by_stage": json.dumps(
                [
                    finite_or_none(value)
                    for value in self.dynamic_min_cost_by_stage
                ],
                separators=(",", ":"),
            ),
            "escape_safe_count": self.escape_safe_count,
            "escape_min_cost": finite_or_none(self.escape_min_cost),
            "dynamic_object_count": len(self.dynamic_objects),
            "dynamic_min_clearance": finite_or_none(
                self._dynamic_min_clearance()
            ),
            "prediction_marker_count": self.prediction_marker_count,
            "prediction_labels": "|".join(self.prediction_labels),
            "local_grid_occupied": self.local_grid_occupied,
            "global_path_poses": (
                len(self.global_path.poses) if self.global_path else 0
            ),
            "local_path_poses": local_path_count,
            "local_path_end_x": local_end_x,
            "local_path_end_y": local_end_y,
            "stop_candidate": int(stop_candidate),
            "stop_cause": cause if stop_candidate else "",
        }
        self.timeline_writer.writerow(row)

    def _finish_session(self, result):
        if self.session is None:
            return
        now = self._sim_time()
        self._close_stop_episode(now)
        summary = dict(self.session)
        summary.update(
            {
                "result": result,
                "end_sim_time": now,
                "duration": max(
                    0.0, now - self.session["start_sim_time"]
                ),
                "end_wall_time": datetime.now().isoformat(
                    timespec="milliseconds"
                ),
                "stop_episodes": self.stop_episodes,
                "stop_cause_totals": {},
            }
        )
        for episode in self.stop_episodes:
            cause = episode["cause"]
            summary["stop_cause_totals"][cause] = (
                summary["stop_cause_totals"].get(cause, 0.0)
                + episode["duration"]
            )
        summary_path = os.path.join(
            self.session["directory"], "summary.json"
        )
        with open(summary_path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=True)
        directory = self.session["directory"]
        for handle in (
            self.timeline_handle,
            self.events_handle,
            self.objects_handle,
        ):
            if handle is not None:
                handle.flush()
                handle.close()
        self.timeline_handle = None
        self.timeline_writer = None
        self.events_handle = None
        self.events_writer = None
        self.objects_handle = None
        self.session = None
        self.stationary_since = None
        self.active_stop_episode = None
        self.get_logger().info(
            "V2 goal diagnostic session finished: result=%s directory=%s"
            % (result, directory)
        )
        self.status_publisher.publish(
            String(
                data="finished result=%s directory=%s"
                % (result, directory)
            )
        )

    def close(self):
        if self.session is not None:
            self._finish_session("diagnostic_node_shutdown")


def main(args=None):
    rclpy.init(args=args)
    node = GoalDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
