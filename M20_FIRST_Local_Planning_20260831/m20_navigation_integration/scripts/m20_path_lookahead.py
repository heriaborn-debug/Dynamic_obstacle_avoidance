#!/usr/bin/env python3
"""Convert any standard global nav_msgs/Path into the V2 planner's local goal."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathLookahead(Node):
    def __init__(self) -> None:
        super().__init__("m20_path_lookahead")
        path_topic = self.declare_parameter(
            "global_path_topic", "/m20/navigation/global_path").value
        odom_topic = self.declare_parameter("odom_topic", "/lio/robo/odom").value
        goal_topic = self.declare_parameter("local_goal_topic", "/local_goal").value
        self.lookahead = float(self.declare_parameter("lookahead_distance", 2.5).value)
        self.republish_distance = float(self.declare_parameter("republish_distance", 0.12).value)
        self.heartbeat = float(self.declare_parameter("heartbeat_period", 1.0).value)
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(Path, path_topic, self.on_path, transient)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.path: Path | None = None
        self.odom: Odometry | None = None
        self.last_goal: tuple[float, float] | None = None
        self.last_publish = 0.0
        self.create_timer(0.10, self.tick)
        self.get_logger().info(
            f"path={path_topic}, odom={odom_topic}, local_goal={goal_topic}, lookahead={self.lookahead:.2f}m")

    def on_path(self, msg: Path) -> None:
        self.path = msg if msg.poses else None
        self.last_goal = None

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def tick(self) -> None:
        if self.path is None or self.odom is None or not self.path.poses:
            return
        robot = self.odom.pose.pose.position
        points = self.path.poses
        nearest = min(
            range(len(points)),
            key=lambda i: (points[i].pose.position.x - robot.x) ** 2 +
                          (points[i].pose.position.y - robot.y) ** 2,
        )
        selected = nearest
        distance = 0.0
        while selected + 1 < len(points) and distance < self.lookahead:
            a = points[selected].pose.position
            b = points[selected + 1].pose.position
            distance += math.hypot(b.x - a.x, b.y - a.y)
            selected += 1
        source = points[selected]
        xy = (source.pose.position.x, source.pose.position.y)
        now = time.monotonic()
        changed = self.last_goal is None or math.hypot(xy[0] - self.last_goal[0], xy[1] - self.last_goal[1]) >= self.republish_distance
        if not changed and now - self.last_publish < self.heartbeat:
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.path.header.frame_id
        goal.pose = source.pose
        self.goal_pub.publish(goal)
        self.last_goal = xy
        self.last_publish = now


def main() -> None:
    rclpy.init()
    node = PathLookahead()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
