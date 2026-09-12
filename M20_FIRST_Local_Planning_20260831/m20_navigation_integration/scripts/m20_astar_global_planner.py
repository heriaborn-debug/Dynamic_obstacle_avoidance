#!/usr/bin/env python3
"""Temporary grid A* global planner with a standard nav_msgs/Path boundary."""

from __future__ import annotations

import heapq
import math
import time
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


class AStarGlobalPlanner(Node):
    def __init__(self) -> None:
        super().__init__("m20_astar_global_planner")
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.odom_topic = self.declare_parameter("odom_topic", "/lio/robo/odom").value
        self.goal_topics = list(self.declare_parameter(
            "goal_topics", ["/goal_pose", "/target_goal", "/move_base_simple/goal"]
        ).value)
        self.path_topic = self.declare_parameter(
            "global_path_topic", "/m20/navigation/global_path").value
        status_topic = self.declare_parameter(
            "status_topic", "/m20/navigation/global_planner_status").value
        self.target_resolution = float(self.declare_parameter("planning_resolution", 0.10).value)
        self.inflation_radius = float(self.declare_parameter("inflation_radius", 0.45).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 65).value)
        self.unknown_occupied = bool(self.declare_parameter("unknown_is_occupied", True).value)
        self.nearest_free_radius = float(self.declare_parameter("nearest_free_radius", 0.75).value)
        self.replan_distance = float(self.declare_parameter("replan_distance", 0.50).value)
        self.path_spacing = float(self.declare_parameter("path_spacing", 0.15).value)
        self.max_planning_time = float(self.declare_parameter("maximum_planning_time", 4.0).value)

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.path_topic, transient)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, transient)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        for topic in self.goal_topics:
            self.create_subscription(PoseStamped, topic, self.on_goal, 10)

        self.map_msg: OccupancyGrid | None = None
        self.blocked: np.ndarray | None = None
        self.resolution = 0.0
        self.factor = 1
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.odom: Odometry | None = None
        self.goal: PoseStamped | None = None
        self.last_plan_start: tuple[float, float] | None = None
        self.plan_requested = False
        self.create_timer(0.25, self.tick)
        self.get_logger().info(
            f"waiting for map={self.map_topic}, odom={self.odom_topic}; output={self.path_topic}")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_map(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return
        self.map_msg = msg
        self.factor = max(1, int(round(self.target_resolution / msg.info.resolution)))
        self.resolution = msg.info.resolution * self.factor
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_yaw = yaw_from_quaternion(msg.info.origin.orientation)
        raw = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        base_blocked = raw >= self.occupied_threshold
        if self.unknown_occupied:
            base_blocked |= raw < 0
        pad_h = (-base_blocked.shape[0]) % self.factor
        pad_w = (-base_blocked.shape[1]) % self.factor
        padded = np.pad(base_blocked, ((0, pad_h), (0, pad_w)), constant_values=True)
        coarse = padded.reshape(
            padded.shape[0] // self.factor, self.factor,
            padded.shape[1] // self.factor, self.factor,
        ).any(axis=(1, 3))
        self.blocked = self.inflate(coarse, self.inflation_radius)
        self.plan_requested = self.goal is not None
        self.publish_status(
            f"map_ready frame={msg.header.frame_id} size={self.blocked.shape[1]}x{self.blocked.shape[0]} "
            f"resolution={self.resolution:.3f} inflation={self.inflation_radius:.2f}")

    def inflate(self, occupied: np.ndarray, radius_m: float) -> np.ndarray:
        radius = int(math.ceil(radius_m / self.resolution))
        if radius <= 0:
            return occupied.copy()
        result = occupied.copy()
        h, w = occupied.shape
        for dy in range(-radius, radius + 1):
            span = int(math.floor(math.sqrt(max(0, radius * radius - dy * dy))))
            src_y0, src_y1 = max(0, -dy), min(h, h - dy)
            dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
            for dx in range(-span, span + 1):
                src_x0, src_x1 = max(0, -dx), min(w, w - dx)
                dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
                result[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[src_y0:src_y1, src_x0:src_x1]
        return result

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        if self.goal is not None and self.last_plan_start is not None:
            p = msg.pose.pose.position
            if math.hypot(p.x - self.last_plan_start[0], p.y - self.last_plan_start[1]) >= self.replan_distance:
                self.plan_requested = True

    def on_goal(self, msg: PoseStamped) -> None:
        map_frame = self.map_msg.header.frame_id if self.map_msg else "world"
        if msg.header.frame_id and msg.header.frame_id != map_frame:
            self.publish_status(f"goal_rejected frame={msg.header.frame_id}, expected={map_frame}")
            return
        self.goal = msg
        self.plan_requested = True
        self.publish_status(f"goal_received x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}")

    def tick(self) -> None:
        if not self.plan_requested or self.blocked is None or self.odom is None or self.goal is None:
            return
        self.plan_requested = False
        self.make_plan()

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(mx / self.resolution)), int(math.floor(my / self.resolution))

    def grid_to_world(self, x: int, y: int) -> tuple[float, float]:
        mx, my = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * mx - s * my, self.origin_y + s * mx + c * my

    def nearest_free(self, cell: tuple[int, int]) -> tuple[int, int] | None:
        assert self.blocked is not None
        x, y = cell
        h, w = self.blocked.shape
        if 0 <= x < w and 0 <= y < h and not self.blocked[y, x]:
            return cell
        max_radius = int(math.ceil(self.nearest_free_radius / self.resolution))
        best = None
        best_d2 = math.inf
        for yy in range(max(0, y - max_radius), min(h, y + max_radius + 1)):
            for xx in range(max(0, x - max_radius), min(w, x + max_radius + 1)):
                if self.blocked[yy, xx]:
                    continue
                d2 = (xx - x) ** 2 + (yy - y) ** 2
                if d2 < best_d2:
                    best, best_d2 = (xx, yy), d2
        return best

    @staticmethod
    def heuristic(x: int, y: int, gx: int, gy: int) -> float:
        dx, dy = abs(gx - x), abs(gy - y)
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        assert self.blocked is not None
        h, w = self.blocked.shape
        total = h * w
        gscore = np.full(total, np.inf, dtype=np.float32)
        came = np.full(total, -1, dtype=np.int64)
        closed = np.zeros(total, dtype=np.bool_)
        sx, sy = start
        gx, gy = goal
        start_i, goal_i = sy * w + sx, gy * w + gx
        gscore[start_i] = 0.0
        queue = [(self.heuristic(sx, sy, gx, gy), 0.0, start_i)]
        neighbors = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                     (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                     (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
        deadline = time.monotonic() + self.max_planning_time
        while queue:
            if time.monotonic() > deadline:
                return None
            _, current_g, current = heapq.heappop(queue)
            if closed[current] or current_g > float(gscore[current]) + 1e-5:
                continue
            if current == goal_i:
                cells = []
                while current >= 0:
                    cells.append((current % w, current // w))
                    current = int(came[current])
                return list(reversed(cells))
            closed[current] = True
            x, y = current % w, current // w
            for dx, dy, cost in neighbors:
                nx, ny = x + dx, y + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h or self.blocked[ny, nx]:
                    continue
                if dx and dy and (self.blocked[y, nx] or self.blocked[ny, x]):
                    continue
                nxt = ny * w + nx
                candidate = current_g + cost
                if candidate + 1e-5 < float(gscore[nxt]):
                    gscore[nxt] = candidate
                    came[nxt] = current
                    heapq.heappush(queue, (candidate + self.heuristic(nx, ny, gx, gy), candidate, nxt))
        return None

    def line_free(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        assert self.blocked is not None
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx - dy
        while True:
            if self.blocked[y0, x0]:
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def simplify(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = min(len(cells) - 1, anchor + 80)
            while candidate > anchor + 1 and not self.line_free(cells[anchor], cells[candidate]):
                candidate -= 1
            result.append(cells[candidate])
            anchor = candidate
        return result

    def densify(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        result = [points[0]]
        for a, b in zip(points, points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(math.ceil(length / self.path_spacing)))
            for i in range(1, steps + 1):
                t = i / steps
                result.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        return result

    def make_plan(self) -> None:
        assert self.odom is not None and self.goal is not None and self.map_msg is not None
        start_world = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        goal_world = (self.goal.pose.position.x, self.goal.pose.position.y)
        start = self.nearest_free(self.world_to_grid(*start_world))
        goal = self.nearest_free(self.world_to_grid(*goal_world))
        if start is None or goal is None:
            self.publish_status("planning_failed: start or goal has no nearby free cell")
            return
        began = time.monotonic()
        cells = self.astar(start, goal)
        if not cells:
            self.publish_status("planning_failed: A* found no path within deadline")
            return
        cells = self.simplify(cells)
        points = self.densify([self.grid_to_world(x, y) for x, y in cells])
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_msg.header.frame_id
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point
            if index + 1 < len(points):
                nxt = points[index + 1]
                pose.pose.orientation = quaternion_from_yaw(math.atan2(nxt[1] - point[1], nxt[0] - point[0]))
            else:
                pose.pose.orientation = self.goal.pose.orientation
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.last_plan_start = start_world
        self.publish_status(
            f"plan_ready poses={len(path.poses)} length_cells={len(cells)} time={time.monotonic()-began:.3f}s")


def main() -> None:
    rclpy.init()
    node = AStarGlobalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
