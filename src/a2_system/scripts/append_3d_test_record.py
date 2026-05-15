#!/usr/bin/env python3
"""Append one 3D navigation test run to the industrial CSV ledger."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CSV = REPO_ROOT / "runtime" / "test_records" / "industrial_3d_nav_runs.csv"

FIELDS = [
    "record_time",
    "run_id",
    "run_type",
    "robot_id",
    "site",
    "operator",
    "software_ref",
    "command",
    "map_id",
    "map_path",
    "bag_path",
    "environment",
    "start_time",
    "end_time",
    "duration_sec",
    "result",
    "failure_reason",
    "stages_completed",
    "ros_domain_id",
    "pointcloud_topic",
    "odom_topic",
    "localization_topic",
    "goal_topic",
    "cmd_topic",
    "seed_points",
    "accumulated_points",
    "loaded_points",
    "ndt_score",
    "ndt_ready_pct",
    "localization_drop_count",
    "mean_xy_error_m",
    "ate_rmse_m",
    "path_waypoints",
    "distance_commanded_m",
    "distance_moved_m",
    "waypoint_success_rate_pct",
    "collision_count",
    "near_collision_count",
    "estop_count",
    "recovery_count",
    "max_cpu_pct",
    "max_mem_mb",
    "battery_start_pct",
    "battery_end_pct",
    "params_profile",
    "artifacts_dir",
    "logs_dir",
    "report_json",
    "notes",
    "next_action",
]


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _parse_time(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value)
    except ValueError:
        return None


def _distance(records: list[dict[str, Any]]) -> str:
    total = 0.0
    for item in records:
        try:
            total += float(item.get("moved_m", 0.0))
        except (TypeError, ValueError):
            continue
    return f"{total:.3f}" if math.isfinite(total) and total > 0.0 else ""


def _load_report(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"report must be a JSON object: {path}")
    return data


def _row_from_report(report: dict[str, Any], report_path: Path | None) -> dict[str, str]:
    row = {field: "" for field in FIELDS}
    stages = report.get("stages", [])
    if not isinstance(stages, list):
        stages = []

    stage_by_name = {
        stage.get("name"): stage for stage in stages if isinstance(stage, dict) and stage.get("name")
    }
    first_time = _parse_time(stages[0].get("time")) if stages else None
    last_time = _parse_time(stages[-1].get("time")) if stages else None
    movement_records = stage_by_name.get("stage5_two_steps_executed", {}).get("movement_records", [])
    if not isinstance(movement_records, list):
        movement_records = []

    artifacts = report.get("artifacts", {})
    if not isinstance(artifacts, dict):
        artifacts = {}

    run_dir = str(report.get("run_dir", ""))
    map_saved = stage_by_name.get("stage3_map_saved", {})
    localized = stage_by_name.get("stage4_map_loaded_and_localized", {})
    planning = stage_by_name.get("stage4_planning_done", {})
    mapping = stage_by_name.get("stage2_mapping_done", {})
    score = localized.get("map_usage_score", {})
    if not isinstance(score, dict):
        score = {}

    row.update(
        {
            "record_time": _utc_now(),
            "run_id": Path(run_dir).name if run_dir else "",
            "run_type": "sim_contract" if "kinematics_sim" in json.dumps(stages) else "",
            "map_id": str(report.get("map_id", "")),
            "map_path": str(artifacts.get("saved_pcd") or map_saved.get("saved_pcd", "")),
            "start_time": first_time.isoformat() if first_time else "",
            "end_time": last_time.isoformat() if last_time else "",
            "duration_sec": f"{(last_time - first_time).total_seconds():.3f}"
            if first_time and last_time
            else "",
            "result": str(report.get("result", "")),
            "failure_reason": str(report.get("error") or stage_by_name.get("stage1_chain", {}).get("known_break", "")),
            "stages_completed": "|".join(str(stage.get("name", "")) for stage in stages if isinstance(stage, dict)),
            "ros_domain_id": str(report.get("ros_domain_id", "")),
            "pointcloud_topic": "/jt128/front/points",
            "odom_topic": "/jt128/dlio/odom",
            "localization_topic": "/a2/relocalization/pose",
            "goal_topic": "/a2/nav3/goal_pose",
            "cmd_topic": "/cmd_vel_safe",
            "seed_points": str(report.get("seed_points", "")),
            "accumulated_points": str(mapping.get("accumulated_points", "")),
            "loaded_points": str(localized.get("loaded_points", "")),
            "mean_xy_error_m": str(score.get("mean_xy_error_m", "")),
            "path_waypoints": str(planning.get("path_waypoints", "")),
            "distance_moved_m": _distance(movement_records),
            "collision_count": "0" if report.get("result") == "PASS" else "",
            "near_collision_count": "0" if report.get("result") == "PASS" else "",
            "params_profile": "slam_3d.yaml|localization_3d.yaml|scan_mission_3d.yaml|nav2_3d.yaml",
            "artifacts_dir": run_dir,
            "logs_dir": str(artifacts.get("logs", "")),
            "report_json": str(report_path) if report_path else "",
        }
    )
    return row


def _merge_cli(row: dict[str, str], args: argparse.Namespace) -> dict[str, str]:
    overrides = {
        "run_id": args.run_id,
        "run_type": args.run_type,
        "robot_id": args.robot_id,
        "site": args.site,
        "operator": args.operator,
        "software_ref": args.software_ref,
        "command": args.command,
        "map_id": args.map_id,
        "bag_path": args.bag_path,
        "environment": args.environment,
        "result": args.result,
        "ndt_score": args.ndt_score,
        "ate_rmse_m": args.ate_rmse_m,
        "waypoint_success_rate_pct": args.waypoint_success_rate_pct,
        "collision_count": args.collision_count,
        "near_collision_count": args.near_collision_count,
        "estop_count": args.estop_count,
        "recovery_count": args.recovery_count,
        "battery_start_pct": args.battery_start_pct,
        "battery_end_pct": args.battery_end_pct,
        "notes": args.notes,
        "next_action": args.next_action,
    }
    for key, value in overrides.items():
        if value is not None:
            row[key] = str(value)
    if not row["record_time"]:
        row["record_time"] = _utc_now()
    return row


def _append_csv(csv_path: Path, row: dict[str, str]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    exists = csv_path.exists() and csv_path.stat().st_size > 0
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in FIELDS})


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Append a 3D navigation run to runtime/test_records/industrial_3d_nav_runs.csv"
    )
    parser.add_argument("--csv", default=str(DEFAULT_CSV), help="CSV ledger path")
    parser.add_argument("--report-json", help="closed_loop_report.json or preflight/result JSON")
    parser.add_argument("--run-id")
    parser.add_argument("--run-type", choices=["real_robot", "rosbag", "sim_contract", "dry_run", "bench"])
    parser.add_argument("--robot-id")
    parser.add_argument("--site")
    parser.add_argument("--operator")
    parser.add_argument("--software-ref", help="git SHA, branch, or release tag")
    parser.add_argument("--command", help="launch/script command used for this run")
    parser.add_argument("--map-id")
    parser.add_argument("--bag-path")
    parser.add_argument("--environment", help="e.g. indoor lab, outdoor asphalt, low light")
    parser.add_argument("--result", choices=["PASS", "FAIL", "ABORTED", "PARTIAL"])
    parser.add_argument("--ndt-score")
    parser.add_argument("--ate-rmse-m")
    parser.add_argument("--waypoint-success-rate-pct")
    parser.add_argument("--collision-count")
    parser.add_argument("--near-collision-count")
    parser.add_argument("--estop-count")
    parser.add_argument("--recovery-count")
    parser.add_argument("--battery-start-pct")
    parser.add_argument("--battery-end-pct")
    parser.add_argument("--notes")
    parser.add_argument("--next-action")
    args = parser.parse_args()

    report_path = Path(args.report_json).resolve() if args.report_json else None
    report = _load_report(report_path)
    row = _merge_cli(_row_from_report(report, report_path), args)
    _append_csv(Path(args.csv).resolve(), row)
    print(f"appended 3D navigation test record: {Path(args.csv).resolve()}")
    print(f"run_id={row.get('run_id', '')} result={row.get('result', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
