from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "auto_scan_mission.py"
    spec = importlib.util.spec_from_file_location("auto_scan_mission_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mission = load_module()


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_node():
    node = mission.AutoScanMission.__new__(mission.AutoScanMission)
    node.mission_name = "test"
    node.validate_waypoints_against_map = True
    node.allow_unknown_cells = False
    node.occupied_threshold = 65
    node.min_clearance_cells = 0
    node.goal_frame = "map"
    node.require_map_frame = True
    node.latest_map = None
    node.latest_pose = None
    node.report_entries = []
    node.route_validation_entries = []
    node.localization_drop_events = 0
    node.real_not_ready_events = 0
    node.localization_ok = True
    node.localization_status_raw = ""
    node.real_report_raw = ""
    node.nav_status_raw = ""
    node.last_feedback_distance = None
    node.total_waypoints = 1
    node.position_pass_threshold_m = 0.12
    node.position_warn_threshold_m = 0.20
    node.yaw_pass_threshold_rad = 0.15
    node.yaw_warn_threshold_rad = 0.30
    node.progress_pub = FakePublisher()
    node.publish_status = lambda *args, **kwargs: None
    return node


def make_map(width=4, height=4, resolution=1.0, data=None):
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = data if data is not None else [0] * (width * height)
    return msg


def make_pose(x, y, yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def test_load_waypoints_normalizes_and_preserves_ids(tmp_path):
    path = tmp_path / "route.yaml"
    path.write_text(
        """
mission_name: lab
waypoints:
  - id: wp1
    x: 1
    y: 2
    yaw: 6.5
    dwell_sec: 1.0
""",
        encoding="utf-8",
    )
    node = make_node()
    node.waypoints_file = str(path)
    node.mission_name = "auto_scan"

    waypoints = node.load_waypoints()

    assert node.mission_name == "lab"
    assert waypoints[0].waypoint_id == "wp1"
    assert -math.pi <= waypoints[0].yaw <= math.pi


def test_load_waypoints_rejects_duplicate_id(tmp_path):
    path = tmp_path / "route.yaml"
    path.write_text(
        """
waypoints:
  - id: same
    x: 0
    y: 0
  - id: same
    x: 1
    y: 1
""",
        encoding="utf-8",
    )
    node = make_node()
    node.waypoints_file = str(path)

    try:
        node.load_waypoints()
    except RuntimeError as exc:
        assert "duplicate waypoint id" in str(exc)
    else:
        raise AssertionError("duplicate waypoint id was not rejected")


def test_world_to_map_cell_and_free_cell_validation():
    node = make_node()
    node.latest_map = make_map()
    waypoint = mission.WaypointSpec("free", 1.2, 2.4, 0.0, 0.0, "")

    assert node.world_to_map_cell(1.2, 2.4) == (1, 2)
    valid, reason, details = node.validate_waypoint_against_map(waypoint)

    assert valid is True
    assert reason == "map_cell_free"
    assert details["map_cell"] == (1, 2)


def test_occupied_and_unknown_cells_are_blocked():
    data = [0] * 16
    data[5] = 90
    data[6] = -1
    node = make_node()
    node.latest_map = make_map(data=data)

    occupied = mission.WaypointSpec("occupied", 1.1, 1.1, 0.0, 0.0, "")
    unknown = mission.WaypointSpec("unknown", 2.1, 1.1, 0.0, 0.0, "")

    assert node.validate_waypoint_against_map(occupied)[1] == "occupied_cell_blocked"
    assert node.validate_waypoint_against_map(unknown)[1] == "unknown_cell_blocked"


def test_finish_waypoint_result_pass_warn_fail_and_missing_pose_fail():
    waypoint = mission.WaypointSpec("goal", 1.0, 1.0, 0.0, 0.0, "")

    node = make_node()
    node.latest_pose = make_pose(1.05, 1.04, 0.05)
    result = node.finish_waypoint_result(waypoint, "succeeded", True, 0.0, 0, 0)
    assert result["validation"] == "pass"

    node = make_node()
    node.latest_pose = make_pose(1.16, 1.0, 0.2)
    result = node.finish_waypoint_result(waypoint, "succeeded", True, 0.0, 0, 0)
    assert result["validation"] == "warn"

    node = make_node()
    node.latest_pose = make_pose(1.5, 1.0, 0.0)
    result = node.finish_waypoint_result(waypoint, "succeeded", True, 0.0, 0, 0)
    assert result["validation"] == "fail"

    node = make_node()
    result = node.finish_waypoint_result(waypoint, "succeeded", True, 0.0, 0, 0)
    assert result["validation"] == "fail"
