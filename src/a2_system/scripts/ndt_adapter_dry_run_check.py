#!/usr/bin/env python3

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def parse_status(status: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for part in status.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        result[key.strip()] = value.strip()
    return result


class NdtAdapterDryRunCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ndt_adapter_dry_run_check")
        self.args = args
        self.start_time = self.get_clock().now()
        self.status = ""
        self.status_time = None
        self.pose: PoseWithCovarianceStamped | None = None
        self.pose_time = None
        self.localization_ok: bool | None = None
        self.localization_ok_time = None
        self.map_time = None
        self.map_points = 0
        self.odom_time = None
        self.live_cloud_time = None
        self.live_cloud_points = 0

        self.create_subscription(String, args.status_topic, self.on_status, 10)
        self.create_subscription(PoseWithCovarianceStamped, args.pose_topic, self.on_pose, 10)
        self.create_subscription(Bool, args.localization_ok_topic, self.on_localization_ok, 10)
        self.create_subscription(PointCloud2, args.map_topic, self.on_map, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(PointCloud2, args.live_cloud_topic, self.on_live_cloud, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_status(self, msg: String) -> None:
        self.status = msg.data
        self.status_time = self.get_clock().now()

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg
        self.pose_time = self.get_clock().now()

    def on_localization_ok(self, msg: Bool) -> None:
        self.localization_ok = bool(msg.data)
        self.localization_ok_time = self.get_clock().now()

    def on_map(self, msg: PointCloud2) -> None:
        self.map_time = self.get_clock().now()
        self.map_points = int(msg.width) * int(msg.height)

    def on_odom(self, _msg: Odometry) -> None:
        self.odom_time = self.get_clock().now()

    def on_live_cloud(self, msg: PointCloud2) -> None:
        self.live_cloud_time = self.get_clock().now()
        self.live_cloud_points = int(msg.width) * int(msg.height)

    def fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def tf_available(self) -> tuple[bool, str]:
        if self.args.skip_tf:
            return True, "skipped"
        try:
            self.tf_buffer.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=0.2))
            return True, "available"
        except TransformException as exc:
            return False, str(exc)

    def evaluate(self) -> tuple[bool, list[str], dict[str, str]]:
        checks: list[tuple[str, bool, str]] = []
        parsed_status = parse_status(self.status)
        checks.append(("map_received", self.map_points > 0 and self.fresh(self.map_time, self.args.topic_timeout_sec), f"points={self.map_points}"))
        checks.append(("odom_fresh", self.fresh(self.odom_time, self.args.topic_timeout_sec), "DLIO odom"))
        checks.append(("live_cloud_fresh", self.live_cloud_points > 0 and self.fresh(self.live_cloud_time, self.args.topic_timeout_sec), f"points={self.live_cloud_points}"))
        checks.append(("status_fresh", self.fresh(self.status_time, self.args.topic_timeout_sec), self.status or "missing"))
        checks.append(("status_matcher", parsed_status.get("matcher") == "autoware_ndt", parsed_status.get("matcher", "missing")))
        checks.append(("status_ready", parsed_status.get("ready") == "true", parsed_status.get("reason", "missing")))
        checks.append(("score_present", "score" in parsed_status, parsed_status.get("score", "missing")))
        checks.append(("iteration_present", "iteration_num" in parsed_status, parsed_status.get("iteration_num", "missing")))
        pose_ok = self.pose is not None and self.pose.header.frame_id == "map" and self.fresh(self.pose_time, self.args.topic_timeout_sec)
        pose_detail = "missing" if self.pose is None else f"frame={self.pose.header.frame_id}"
        checks.append(("pose_fresh_map_frame", pose_ok, pose_detail))
        if self.args.require_localization_ok:
            checks.append(("localization_ok", self.localization_ok is True and self.fresh(self.localization_ok_time, self.args.topic_timeout_sec), str(self.localization_ok)))
        tf_ok, tf_detail = self.tf_available()
        checks.append(("tf_map_to_odom", tf_ok, tf_detail))

        lines = [f"- {name}: {'PASS' if ok else 'FAIL'} ({detail})" for name, ok, detail in checks]
        return all(ok for _, ok, _ in checks), lines, parsed_status


def write_report(args: argparse.Namespace, passed: bool, lines: list[str], parsed_status: dict[str, str]) -> Path:
    report_dir = Path(args.report_dir).expanduser()
    report_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = report_dir / f"ndt_adapter_dry_run_{stamp}.md"
    path.write_text(
        "\n".join(
            [
                "# NDT Adapter Dry-Run Check",
                "",
                f"- Result: {'PASS' if passed else 'FAIL'}",
                f"- Live cloud topic: `{args.live_cloud_topic}`",
                f"- Odom topic: `{args.odom_topic}`",
                f"- Pose topic: `{args.pose_topic}`",
                f"- Status topic: `{args.status_topic}`",
                "",
                "## Checks",
                *lines,
                "",
                "## Parsed Status",
                *[f"- {key}: `{value}`" for key, value in sorted(parsed_status.items())],
                "",
            ]
        ),
        encoding="utf-8",
    )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description="Dry-run validation for A2 Autoware NDT adapter readiness.")
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--topic-timeout-sec", type=float, default=2.0)
    parser.add_argument("--report-dir", default="runtime/reports")
    parser.add_argument("--live-cloud-topic", default="/jt128/front/points")
    parser.add_argument("--odom-topic", default="/jt128/dlio/odom")
    parser.add_argument("--map-topic", default="/a2/map/pointcloud_3d")
    parser.add_argument("--pose-topic", default="/a2/relocalization/pose")
    parser.add_argument("--status-topic", default="/a2/relocalization/status")
    parser.add_argument("--localization-ok-topic", default="/a2/localization_ok")
    parser.add_argument("--skip-tf", action="store_true")
    parser.add_argument("--allow-localization-not-ok", action="store_false", dest="require_localization_ok")
    parser.set_defaults(require_localization_ok=True)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = NdtAdapterDryRunCheck(args)
    passed = False
    lines: list[str] = []
    parsed_status: dict[str, str] = {}
    try:
        deadline = node.get_clock().now() + Duration(seconds=args.timeout_sec)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            passed, lines, parsed_status = node.evaluate()
            if passed:
                break
        passed, lines, parsed_status = node.evaluate()
        report = write_report(args, passed, lines, parsed_status)
        print(f"{'PASS' if passed else 'FAIL'}: NDT adapter dry-run check report={report}")
        for line in lines:
            print(line)
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
