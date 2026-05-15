#!/usr/bin/env python3
"""Visualize and record A/B/C localization-navigation isolation experiments."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


CSV_FIELDS = [
    "record_time",
    "run_id",
    "phase_label",
    "experiment_mode",
    "result",
    "reference_label",
    "reference_source",
    "map_id",
    "pose_topic",
    "goal_topic",
    "status_topic",
    "anchor_file",
    "notes",
    "start_x",
    "start_y",
    "start_yaw_deg",
    "reference_x",
    "reference_y",
    "reference_yaw_deg",
    "final_x",
    "final_y",
    "final_yaw_deg",
    "pos_error_m",
    "yaw_error_deg",
    "path_length_m",
    "duration_sec",
    "sample_count",
    "status_state",
    "status_reason",
    "status_ready",
]


TERMINAL_NAV_STATES = {
    "goal_succeeded": "success",
    "goal_aborted": "aborted",
    "goal_failed": "failed",
    "goal_canceled": "canceled",
    "goal_timeout": "timeout",
}


COMPARE_MODES = {"relocalization", "anchor_compare"}


def _workspace_root() -> Path:
    raw = os.environ.get("A2_WORKSPACE", "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    script_path = Path(__file__).resolve()
    for parent in script_path.parents:
        if parent.name == "install" and (parent.parent / "src").is_dir():
            return parent.parent
        if (parent / "src").is_dir() and (parent / "runtime").is_dir():
            return parent
    return script_path.parents[3]


WORKSPACE_ROOT = _workspace_root()
DEFAULT_CSV_PATH = WORKSPACE_ROOT / "runtime" / "test_records" / "abc_isolation_runs.csv"
DEFAULT_ANCHOR_PATH = WORKSPACE_ROOT / "runtime" / "test_records" / "abc_anchor_pose.json"


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_monotonic: float


@dataclass
class ActiveRun:
    run_id: str
    phase_label: str
    start_time_monotonic: float
    reference_label: str
    reference_pose: PoseSample
    start_pose: PoseSample | None
    path_samples: list[PoseSample]
    terminal_state: str = ""
    terminal_reason: str = ""
    status_ready: str = ""


@dataclass
class RunSummary:
    run_id: str
    phase_label: str
    experiment_mode: str
    result: str
    reference_label: str
    reference_source: str
    reference_pose: PoseSample
    final_pose: PoseSample
    start_pose: PoseSample | None
    path_samples: list[PoseSample]
    duration_sec: float
    status_state: str
    status_reason: str
    status_ready: str


def utc_now_text() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def pose_from_pose_stamped(msg: PoseStamped) -> PoseSample:
    q = msg.pose.orientation
    return PoseSample(
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
        frame_id=msg.header.frame_id or "map",
        stamp_monotonic=time.monotonic(),
    )


def pose_from_pose_covariance(msg: PoseWithCovarianceStamped) -> PoseSample:
    q = msg.pose.pose.orientation
    return PoseSample(
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
        frame_id=msg.header.frame_id or "map",
        stamp_monotonic=time.monotonic(),
    )


def pose_from_odometry(msg: Odometry) -> PoseSample:
    q = msg.pose.pose.orientation
    return PoseSample(
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
        frame_id=msg.header.frame_id or "odom",
        stamp_monotonic=time.monotonic(),
    )


def pose_distance(a: PoseSample, b: PoseSample) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def yaw_error_deg(reference: PoseSample, actual: PoseSample) -> float:
    return math.degrees(abs(normalize_angle(actual.yaw - reference.yaw)))


def path_length(samples: list[PoseSample]) -> float:
    total = 0.0
    for index in range(1, len(samples)):
        total += pose_distance(samples[index - 1], samples[index])
    return total


def parse_status_fields(raw: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in raw.split(";"):
        part = token.strip()
        if not part or "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields


def save_anchor(path: Path, pose: PoseSample, *, pose_topic: str, phase_label: str, notes: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "saved_at": utc_now_text(),
        "phase_label": phase_label,
        "pose_topic": pose_topic,
        "notes": notes,
        "pose": {
            "x": pose.x,
            "y": pose.y,
            "yaw": pose.yaw,
            "frame_id": pose.frame_id,
        },
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_anchor(path: Path) -> tuple[PoseSample | None, str]:
    if not path.exists():
        return None, ""
    payload = json.loads(path.read_text(encoding="utf-8"))
    pose_raw = payload.get("pose", {})
    if not isinstance(pose_raw, dict):
        raise ValueError(f"invalid anchor file format: {path}")
    pose = PoseSample(
        x=float(pose_raw.get("x", 0.0)),
        y=float(pose_raw.get("y", 0.0)),
        yaw=float(pose_raw.get("yaw", 0.0)),
        frame_id=str(pose_raw.get("frame_id", "map")),
        stamp_monotonic=time.monotonic(),
    )
    source = str(payload.get("phase_label") or path.name)
    return pose, source


def make_color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def marker_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class IsolationExperimentNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("abc_isolation_eval")
        self.args = args
        self.latest_pose: PoseSample | None = None
        self.latest_goal: PoseSample | None = None
        self.latest_nav_status_raw = ""
        self.latest_nav_status: dict[str, str] = {}
        self.latest_relocalization_status_raw = ""
        self.latest_relocalization_status: dict[str, str] = {}
        self.anchor_pose: PoseSample | None = None
        self.anchor_source = ""
        self.anchor_written = False
        self.active_run: ActiveRun | None = None
        self.last_summary: RunSummary | None = None
        self.relocalization_ready_since: float | None = None
        self.relocalization_recorded = False
        self.within_goal_tolerance_since: float | None = None
        self.done = False
        self.exit_code = 0
        self.last_frame_mismatch_warning = 0.0

        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.path_pub = self.create_publisher(PathMsg, args.path_topic, 10)
        self.create_timer(args.publish_period_sec, self._on_timer)

        if args.pose_msg_type == "pose_with_covariance":
            self.create_subscription(
                PoseWithCovarianceStamped, args.pose_topic, self._on_pose_with_covariance, 20
            )
        else:
            self.create_subscription(Odometry, args.pose_topic, self._on_odometry, 20)

        if args.mode == "navigation":
            self.create_subscription(PoseStamped, args.goal_topic, self._on_goal, 20)
            self.create_subscription(String, args.status_topic, self._on_nav_status, 20)
        elif args.mode in COMPARE_MODES:
            self.create_subscription(
                String, args.status_topic, self._on_relocalization_status, 20
            )
            if args.use_initialpose_as_anchor:
                self.create_subscription(
                    PoseWithCovarianceStamped, args.initialpose_topic, self._on_initialpose, 10
                )
            if args.anchor_file:
                anchor_path = Path(args.anchor_file).expanduser().resolve()
                self.anchor_pose, self.anchor_source = load_anchor(anchor_path)
                if self.anchor_pose is not None:
                    self.get_logger().info(
                        f"Loaded anchor pose from {anchor_path} ({self.anchor_source})."
                    )
                elif not args.use_initialpose_as_anchor:
                    self.get_logger().warning(
                        f"Anchor file not found yet: {anchor_path}. "
                        "Compare-mode markers and CSV will wait for an anchor pose."
                    )
        else:
            self.get_logger().info(
                f"Capture-anchor mode: waiting for a fresh pose on {args.pose_topic}."
            )

    def _on_pose_with_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        self._handle_pose(pose_from_pose_covariance(msg))

    def _on_odometry(self, msg: Odometry) -> None:
        self._handle_pose(pose_from_odometry(msg))

    def _handle_pose(self, pose: PoseSample) -> None:
        self.latest_pose = pose
        if self.args.mode == "capture_anchor":
            self._capture_anchor_if_ready()
            return
        if self.args.mode == "navigation" and self.active_run is not None:
            self._append_path_sample(self.active_run, pose)
            self._check_goal_tolerance_completion()

    def _on_goal(self, msg: PoseStamped) -> None:
        goal_pose = pose_from_pose_stamped(msg)
        self.latest_goal = goal_pose
        if self.active_run is not None:
            self._finalize_navigation_run("interrupted", "new_goal_received")
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        start_pose = self.latest_pose
        path_samples = [start_pose] if start_pose is not None else []
        self.active_run = ActiveRun(
            run_id=run_id,
            phase_label=self.args.phase_label,
            start_time_monotonic=time.monotonic(),
            reference_label="goal",
            reference_pose=goal_pose,
            start_pose=start_pose,
            path_samples=path_samples,
        )
        self.within_goal_tolerance_since = None
        self.get_logger().info(
            f"Started navigation run {run_id} phase={self.args.phase_label} "
            f"goal=({goal_pose.x:.3f}, {goal_pose.y:.3f}, {math.degrees(goal_pose.yaw):.1f}deg)"
        )

    def _on_nav_status(self, msg: String) -> None:
        self.latest_nav_status_raw = msg.data
        self.latest_nav_status = parse_status_fields(msg.data)
        if self.active_run is None:
            return
        state = self.latest_nav_status.get("state", "")
        reason = self.latest_nav_status.get("reason", "")
        ready = self.latest_nav_status.get("ready", "")
        self.active_run.terminal_state = state
        self.active_run.terminal_reason = reason
        self.active_run.status_ready = ready
        if state in TERMINAL_NAV_STATES:
            self._finalize_navigation_run(TERMINAL_NAV_STATES[state], reason or state)

    def _on_relocalization_status(self, msg: String) -> None:
        self.latest_relocalization_status_raw = msg.data
        self.latest_relocalization_status = parse_status_fields(msg.data)
        ready = self.latest_relocalization_status.get("ready", "").lower()
        if ready == "true":
            if self.relocalization_ready_since is None:
                self.relocalization_ready_since = time.monotonic()
        else:
            self.relocalization_ready_since = None
            self.relocalization_recorded = False

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = pose_from_pose_covariance(msg)
        self.anchor_pose = pose
        self.anchor_source = "initialpose"
        self.relocalization_recorded = False
        self.relocalization_ready_since = None
        self.get_logger().info(
            f"Updated relocalization anchor from /initialpose at ({pose.x:.3f}, {pose.y:.3f})."
        )

    def _capture_anchor_if_ready(self) -> None:
        if self.anchor_written or self.latest_pose is None:
            return
        if self._pose_is_stale(self.latest_pose):
            return
        anchor_path = Path(self.args.anchor_file).expanduser().resolve()
        save_anchor(
            anchor_path,
            self.latest_pose,
            pose_topic=self.args.pose_topic,
            phase_label=self.args.phase_label,
            notes=self.args.notes,
        )
        self.anchor_written = True
        self.done = True
        self.exit_code = 0
        self.get_logger().info(
            f"Saved anchor pose to {anchor_path} at ({self.latest_pose.x:.3f}, {self.latest_pose.y:.3f})."
        )

    def _pose_is_stale(self, pose: PoseSample | None) -> bool:
        if pose is None:
            return True
        return (time.monotonic() - pose.stamp_monotonic) > self.args.max_pose_age_sec

    def _append_path_sample(self, run: ActiveRun, pose: PoseSample) -> None:
        if not run.path_samples:
            run.path_samples.append(pose)
            return
        previous = run.path_samples[-1]
        if pose_distance(previous, pose) >= self.args.path_sample_distance_m:
            run.path_samples.append(pose)
            return
        if (pose.stamp_monotonic - previous.stamp_monotonic) >= self.args.path_sample_period_sec:
            run.path_samples.append(pose)

    def _check_goal_tolerance_completion(self) -> None:
        if self.active_run is None or self.latest_pose is None:
            return
        error_m = pose_distance(self.active_run.reference_pose, self.latest_pose)
        error_yaw_deg = yaw_error_deg(self.active_run.reference_pose, self.latest_pose)
        if error_m <= self.args.position_tolerance_m and error_yaw_deg <= self.args.yaw_tolerance_deg:
            if self.within_goal_tolerance_since is None:
                self.within_goal_tolerance_since = time.monotonic()
            elif (
                self.args.auto_finalize_on_tolerance
                and (time.monotonic() - self.within_goal_tolerance_since) >= self.args.goal_settle_sec
            ):
                self._finalize_navigation_run("success", "goal_tolerance_window")
        else:
            self.within_goal_tolerance_since = None

    def _finalize_navigation_run(self, result: str, reason: str) -> None:
        if self.active_run is None:
            return
        final_pose = self.latest_pose or self.active_run.path_samples[-1] if self.active_run.path_samples else None
        if final_pose is None:
            self.get_logger().warning("Navigation run finished without a usable final pose; skipping CSV row.")
            self.active_run = None
            self.within_goal_tolerance_since = None
            return
        summary = RunSummary(
            run_id=self.active_run.run_id,
            phase_label=self.active_run.phase_label,
            experiment_mode="navigation",
            result=result,
            reference_label=self.active_run.reference_label,
            reference_source=self.args.goal_topic,
            reference_pose=self.active_run.reference_pose,
            final_pose=final_pose,
            start_pose=self.active_run.start_pose,
            path_samples=list(self.active_run.path_samples) or [final_pose],
            duration_sec=time.monotonic() - self.active_run.start_time_monotonic,
            status_state=self.active_run.terminal_state or result,
            status_reason=reason,
            status_ready=self.active_run.status_ready,
        )
        self._append_summary(summary)
        self.last_summary = summary
        self.active_run = None
        self.within_goal_tolerance_since = None
        self.get_logger().info(
            f"Recorded navigation run {summary.run_id} result={summary.result} "
            f"pos_error={pose_distance(summary.reference_pose, summary.final_pose):.3f}m "
            f"yaw_error={yaw_error_deg(summary.reference_pose, summary.final_pose):.1f}deg"
        )

    def _record_relocalization_result(self) -> None:
        if self.relocalization_recorded or self.anchor_pose is None or self.latest_pose is None:
            return
        summary = RunSummary(
            run_id=datetime.now().strftime("%Y%m%d_%H%M%S"),
            phase_label=self.args.phase_label,
            experiment_mode=self.args.mode,
            result="ready" if self.args.mode == "relocalization" else "measured",
            reference_label="anchor",
            reference_source=self.anchor_source or self.args.anchor_file,
            reference_pose=self.anchor_pose,
            final_pose=self.latest_pose,
            start_pose=None,
            path_samples=[self.latest_pose],
            duration_sec=(time.monotonic() - self.relocalization_ready_since)
            if self.relocalization_ready_since is not None
            else 0.0,
            status_state=self.latest_relocalization_status.get("state", "ready"),
            status_reason=self.latest_relocalization_status.get("reason", ""),
            status_ready=self.latest_relocalization_status.get("ready", ""),
        )
        self._append_summary(summary)
        self.last_summary = summary
        self.relocalization_recorded = True
        self.get_logger().info(
            f"Recorded relocalization run {summary.run_id} "
            f"anchor_delta={pose_distance(summary.reference_pose, summary.final_pose):.3f}m "
            f"yaw_delta={yaw_error_deg(summary.reference_pose, summary.final_pose):.1f}deg"
        )

    def _append_summary(self, summary: RunSummary) -> None:
        row = {field: "" for field in CSV_FIELDS}
        row.update(
            {
                "record_time": utc_now_text(),
                "run_id": summary.run_id,
                "phase_label": summary.phase_label,
                "experiment_mode": summary.experiment_mode,
                "result": summary.result,
                "reference_label": summary.reference_label,
                "reference_source": summary.reference_source,
                "map_id": self.args.map_id,
                "pose_topic": self.args.pose_topic,
                "goal_topic": self.args.goal_topic,
                "status_topic": self.args.status_topic,
                "anchor_file": self.args.anchor_file,
                "notes": self.args.notes,
                "reference_x": f"{summary.reference_pose.x:.4f}",
                "reference_y": f"{summary.reference_pose.y:.4f}",
                "reference_yaw_deg": f"{math.degrees(summary.reference_pose.yaw):.2f}",
                "final_x": f"{summary.final_pose.x:.4f}",
                "final_y": f"{summary.final_pose.y:.4f}",
                "final_yaw_deg": f"{math.degrees(summary.final_pose.yaw):.2f}",
                "pos_error_m": f"{pose_distance(summary.reference_pose, summary.final_pose):.4f}",
                "yaw_error_deg": f"{yaw_error_deg(summary.reference_pose, summary.final_pose):.2f}",
                "path_length_m": f"{path_length(summary.path_samples):.4f}",
                "duration_sec": f"{summary.duration_sec:.3f}",
                "sample_count": str(len(summary.path_samples)),
                "status_state": summary.status_state,
                "status_reason": summary.status_reason,
                "status_ready": summary.status_ready,
            }
        )
        if summary.start_pose is not None:
            row["start_x"] = f"{summary.start_pose.x:.4f}"
            row["start_y"] = f"{summary.start_pose.y:.4f}"
            row["start_yaw_deg"] = f"{math.degrees(summary.start_pose.yaw):.2f}"
        self._append_csv_row(row)

    def _append_csv_row(self, row: dict[str, str]) -> None:
        csv_path = Path(self.args.csv_path).expanduser().resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        exists = csv_path.exists() and csv_path.stat().st_size > 0
        with csv_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            if not exists:
                writer.writeheader()
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})

    def _marker_frame(self, reference_pose: PoseSample | None, actual_pose: PoseSample | None) -> str:
        marker_frame = (
            self.args.marker_frame.strip()
            or (reference_pose.frame_id if reference_pose is not None else "")
            or (actual_pose.frame_id if actual_pose is not None else "")
            or "map"
        )
        if (
            reference_pose is not None
            and actual_pose is not None
            and reference_pose.frame_id
            and actual_pose.frame_id
            and reference_pose.frame_id != actual_pose.frame_id
        ):
            now = time.monotonic()
            if (now - self.last_frame_mismatch_warning) > 5.0:
                self.last_frame_mismatch_warning = now
                self.get_logger().warning(
                    "Reference frame and pose frame differ; markers assume they are already aligned "
                    f"reference={reference_pose.frame_id} actual={actual_pose.frame_id}"
                )
        return marker_frame

    def _publish_visuals(self) -> None:
        reference_pose: PoseSample | None = None
        actual_pose: PoseSample | None = None
        path_samples: list[PoseSample] = []
        reference_label = ""
        result_label = ""

        if self.active_run is not None:
            reference_pose = self.active_run.reference_pose
            actual_pose = self.latest_pose
            path_samples = list(self.active_run.path_samples)
            reference_label = f"{self.args.phase_label} nav active"
            result_label = self.latest_nav_status.get("state", "running")
        elif self.args.mode == "relocalization" and self.anchor_pose is not None:
            reference_pose = self.anchor_pose
            actual_pose = self.latest_pose
            path_samples = [pose for pose in [self.latest_pose] if pose is not None]
            reference_label = f"{self.args.phase_label} reloc"
            result_label = self.latest_relocalization_status.get("state", "waiting")
        elif self.args.mode == "anchor_compare" and self.anchor_pose is not None:
            reference_pose = self.anchor_pose
            actual_pose = self.latest_pose
            path_samples = [pose for pose in [self.latest_pose] if pose is not None]
            reference_label = f"{self.args.phase_label} slam"
            result_label = self.latest_relocalization_status.get("state", "measuring")
        elif self.last_summary is not None:
            reference_pose = self.last_summary.reference_pose
            actual_pose = self.last_summary.final_pose
            path_samples = list(self.last_summary.path_samples)
            reference_label = f"{self.last_summary.phase_label} {self.last_summary.experiment_mode}"
            result_label = self.last_summary.result
        elif self.args.mode == "capture_anchor" and self.latest_pose is not None:
            reference_pose = self.latest_pose
            actual_pose = self.latest_pose
            path_samples = [self.latest_pose]
            reference_label = f"{self.args.phase_label} anchor"
            result_label = "capture"

        if reference_pose is None and actual_pose is None:
            return

        marker_frame = self._marker_frame(reference_pose, actual_pose)
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))

        if reference_pose is not None:
            markers.markers.append(
                self._make_pose_marker(
                    marker_frame,
                    1,
                    "reference",
                    reference_pose,
                    make_color(0.05, 0.55, 0.95, 0.95),
                )
            )
        if actual_pose is not None:
            markers.markers.append(
                self._make_pose_marker(
                    marker_frame,
                    2,
                    "actual",
                    actual_pose,
                    make_color(0.96, 0.45, 0.08, 0.98),
                )
            )
        if self.last_summary is not None and self.last_summary.start_pose is not None:
            markers.markers.append(
                self._make_pose_marker(
                    marker_frame,
                    3,
                    "start",
                    self.last_summary.start_pose,
                    make_color(0.20, 0.75, 0.25, 0.80),
                )
            )
        if reference_pose is not None and actual_pose is not None:
            markers.markers.append(
                self._make_error_line_marker(marker_frame, 4, reference_pose, actual_pose)
            )
            markers.markers.append(
                self._make_text_marker(
                    marker_frame,
                    5,
                    reference_pose,
                    actual_pose,
                    reference_label=reference_label,
                    result_label=result_label,
                )
            )

        self.marker_pub.publish(markers)
        self.path_pub.publish(self._make_path_message(marker_frame, path_samples))

    def _make_pose_marker(
        self,
        marker_frame: str,
        marker_id: int,
        namespace: str,
        pose: PoseSample,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = self.args.marker_z
        marker.pose.orientation.z = math.sin(pose.yaw / 2.0)
        marker.pose.orientation.w = math.cos(pose.yaw / 2.0)
        marker.scale.x = 0.55
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color = color
        return marker

    def _make_error_line_marker(
        self,
        marker_frame: str,
        marker_id: int,
        reference_pose: PoseSample,
        actual_pose: PoseSample,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "error_line"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color = make_color(0.98, 0.84, 0.12, 0.95)
        marker.points = [
            marker_point(reference_pose.x, reference_pose.y, self.args.marker_z),
            marker_point(actual_pose.x, actual_pose.y, self.args.marker_z),
        ]
        return marker

    def _make_text_marker(
        self,
        marker_frame: str,
        marker_id: int,
        reference_pose: PoseSample,
        actual_pose: PoseSample,
        *,
        reference_label: str,
        result_label: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "error_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = (reference_pose.x + actual_pose.x) * 0.5
        marker.pose.position.y = (reference_pose.y + actual_pose.y) * 0.5
        marker.pose.position.z = self.args.marker_z + 0.55
        marker.scale.z = 0.24
        marker.color = make_color(1.0, 1.0, 1.0, 0.98)
        marker.text = (
            f"{reference_label} {result_label}\n"
            f"xy={pose_distance(reference_pose, actual_pose):.3f}m "
            f"yaw={yaw_error_deg(reference_pose, actual_pose):.1f}deg"
        )
        return marker

    def _make_path_message(self, marker_frame: str, samples: list[PoseSample]) -> PathMsg:
        msg = PathMsg()
        msg.header.frame_id = marker_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for sample in samples:
            pose = PoseStamped()
            pose.header.frame_id = marker_frame
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = sample.x
            pose.pose.position.y = sample.y
            pose.pose.position.z = self.args.marker_z
            pose.pose.orientation.z = math.sin(sample.yaw / 2.0)
            pose.pose.orientation.w = math.cos(sample.yaw / 2.0)
            msg.poses.append(pose)
        return msg

    def _on_timer(self) -> None:
        if self.args.mode == "capture_anchor":
            self._capture_anchor_if_ready()
            self._publish_visuals()
            return
        if (
            self.args.mode == "relocalization"
            and not self.relocalization_recorded
            and self.anchor_pose is not None
            and self.latest_pose is not None
            and self.relocalization_ready_since is not None
            and (time.monotonic() - self.relocalization_ready_since) >= self.args.ready_stable_sec
        ):
            self._record_relocalization_result()
        if (
            self.args.mode == "anchor_compare"
            and not self.relocalization_recorded
            and self.anchor_pose is not None
            and self.latest_pose is not None
            and not self._pose_is_stale(self.latest_pose)
        ):
            self._record_relocalization_result()
        self._publish_visuals()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize and record A/B/C isolation experiments for relocalization and navigation."
    )
    parser.add_argument(
        "--mode",
        choices=["capture_anchor", "anchor_compare", "relocalization", "navigation"],
        default="navigation",
    )
    parser.add_argument("--phase-label", default="A")
    parser.add_argument("--map-id", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument("--pose-topic", default="/a2/relocalization/pose")
    parser.add_argument("--pose-msg-type", choices=["pose_with_covariance", "odometry"], default="pose_with_covariance")
    parser.add_argument("--goal-topic", default="/a2/nav3/goal_pose")
    parser.add_argument("--status-topic", default="/a2/nav2/status")
    parser.add_argument("--initialpose-topic", default="/initialpose")
    parser.add_argument("--marker-topic", default="/a2/experiment/markers")
    parser.add_argument("--path-topic", default="/a2/experiment/path")
    parser.add_argument("--marker-frame", default="map")
    parser.add_argument("--marker-z", type=float, default=0.05)
    parser.add_argument("--csv-path", default=str(DEFAULT_CSV_PATH))
    parser.add_argument("--anchor-file", default=str(DEFAULT_ANCHOR_PATH))
    parser.add_argument("--position-tolerance-m", type=float, default=0.30)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=15.0)
    parser.add_argument("--goal-settle-sec", type=float, default=1.0)
    parser.add_argument("--ready-stable-sec", type=float, default=1.5)
    parser.add_argument("--max-pose-age-sec", type=float, default=2.0)
    parser.add_argument("--path-sample-distance-m", type=float, default=0.05)
    parser.add_argument("--path-sample-period-sec", type=float, default=0.50)
    parser.add_argument("--publish-period-sec", type=float, default=0.20)
    parser.add_argument("--auto-finalize-on-tolerance", action="store_true", default=True)
    parser.add_argument("--no-auto-finalize-on-tolerance", dest="auto_finalize_on_tolerance", action="store_false")
    parser.add_argument("--use-initialpose-as-anchor", action="store_true", default=False)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = IsolationExperimentNode(args)
    try:
        if args.mode == "capture_anchor":
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
            return node.exit_code
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
