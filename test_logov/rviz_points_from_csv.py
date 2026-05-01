#!/usr/bin/env python3
"""
Publish all positions from a drone CSV as RViz2 points.

CSV format (header): t,x,y,z,rx,ry,rz,hasCollision
Coordinates in logs are NED; RViz uses ENU. This script converts NED→ENU
in the same way as Drone_Swarm_Simulator_v2/replay/replay_rviz2.py.
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _ned_to_enu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    # NED: (x_north, y_east, z_down) -> ENU: (x=y_east, y=x_north, z=-z_down)
    return (float(y), float(x), float(-z))


@dataclass(frozen=True)
class CsvPoint:
    t: float
    x: float
    y: float
    z: float
    has_collision: int


def _read_points(csv_path: str) -> List[CsvPoint]:
    out: List[CsvPoint] = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"t", "x", "y", "z"}
        if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
            raise ValueError(
                f"Unexpected CSV header {reader.fieldnames}; expected at least {sorted(required)}"
            )
        for row in reader:
            out.append(
                CsvPoint(
                    t=float(row["t"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    z=float(row["z"]),
                    has_collision=int(row.get("hasCollision", "0") or 0),
                )
            )
    return out


class CsvPointsPublisher(Node):
    def __init__(
        self,
        *,
        csv_path: str,
        frame_id: str,
        topic: str,
        point_size: float,
        stride: int,
        rgba: Tuple[float, float, float, float],
        rgba_collision: Tuple[float, float, float, float],
    ) -> None:
        super().__init__("csv_points_publisher")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Marker, topic, qos)

        pts = _read_points(csv_path)
        if stride < 1:
            stride = 1
        pts = pts[::stride]

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "csv_track"
        marker.id = 1
        marker.action = Marker.ADD
        marker.type = Marker.POINTS

        # POINTS uses scale.x and scale.y as point width/height (meters in RViz).
        s = float(point_size)
        if s <= 0:
            s = 0.05
        marker.scale.x = s
        marker.scale.y = s

        base = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
        coll = ColorRGBA(
            r=float(rgba_collision[0]),
            g=float(rgba_collision[1]),
            b=float(rgba_collision[2]),
            a=float(rgba_collision[3]),
        )

        for p in pts:
            enu_x, enu_y, enu_z = _ned_to_enu(p.x, p.y, p.z)
            marker.points.append(Point(x=enu_x, y=enu_y, z=enu_z))
            marker.colors.append(coll if p.has_collision else base)

        self._marker = marker

        self._timer = self.create_timer(1.0, self._republish)
        self.get_logger().info(
            f"Loaded {len(pts)} points (after stride={stride}). "
            f"Publishing on {topic} (frame_id={frame_id})."
        )

    def _republish(self) -> None:
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._marker)


def _parse_rgba(s: str) -> Tuple[float, float, float, float]:
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("RGBA must be 'r,g,b,a' (4 floats 0..1)")
    return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))


def main() -> None:
    ap = argparse.ArgumentParser(description="Publish CSV positions as RViz2 points.")
    ap.add_argument("csv", help="Path to drone_*.csv")
    ap.add_argument("--frame-id", default="world", help="Marker header frame_id (RViz Fixed Frame).")
    ap.add_argument("--topic", default="/swarm/track_points/drone_1", help="Marker topic.")
    ap.add_argument("--point-size", type=float, default=0.06, help="Point size (meters).")
    ap.add_argument("--stride", type=int, default=1, help="Take every Nth point (subsample).")
    ap.add_argument(
        "--rgba",
        type=_parse_rgba,
        default=(0.2, 0.8, 1.0, 1.0),
        help="Base color r,g,b,a (0..1).",
    )
    ap.add_argument(
        "--rgba-collision",
        type=_parse_rgba,
        default=(1.0, 0.2, 0.2, 1.0),
        help="Color for rows with hasCollision=1.",
    )
    args = ap.parse_args()

    csv_path = os.path.abspath(os.path.expanduser(args.csv))
    if not os.path.exists(csv_path):
        raise SystemExit(f"CSV not found: {csv_path}")

    # Ensure ROS2 logging stays writable even in sandboxed runs.
    os.environ.setdefault("ROS_LOG_DIR", os.path.join(os.getcwd(), ".ros_log"))
    os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)

    rclpy.init()
    node = CsvPointsPublisher(
        csv_path=csv_path,
        frame_id=args.frame_id,
        topic=args.topic,
        point_size=args.point_size,
        stride=args.stride,
        rgba=args.rgba,
        rgba_collision=args.rgba_collision,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

