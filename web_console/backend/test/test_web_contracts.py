from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import load_config
from backend.models import CameraFrame, DashboardSnapshot, MapMediaEntry, MapMediaListing, TaskRouteStatus, VirtualObstacleListing, VirtualObstacleZone
from backend.stack_control import MAPPING_NODES, NAVIGATION_NODES, STACK_CLEANUP_PATTERNS


def test_dashboard_snapshot_contains_camera_contract():
    snapshot = DashboardSnapshot()

    assert isinstance(snapshot.camera, CameraFrame)
    assert snapshot.camera.available is False
    assert snapshot.health.camera_received is False


def test_default_config_exposes_camera_topics():
    config = load_config(Path(__file__).resolve().parents[1] / "config.example.yaml")

    assert config.camera.enabled is True
    assert config.ros.camera_compressed_topic == "/camera/image_raw/compressed"
    assert config.ros.camera_image_topic == "/camera/image_raw"
    assert config.navigation.initial_pose_wait_timeout_sec >= 5.0
    assert config.navigation.initial_pose_publish_interval_sec > 0.0
    assert config.navigation.backend == "nav2"
    assert config.navigation.goal_topic == "/goal_pose_"
    assert config.navigation.cancel_stop_topic == "/cmd_vel"
    assert config.navigation.cancel_retarget_current_pose is True
    assert config.navigation.require_map_for_goal is True
    assert config.native_slam.enabled is True
    assert config.native_slam.request_topic == "/api/slam_operate/request"
    assert config.native_slam.response_topic == "/api/slam_operate/response"
    assert config.native_slam.response_timeout_sec >= 1.0
    assert config.ros.pointcloud_topic == "/jt128/front/points"
    assert config.ros.pointcloud_fallback_topic == "/jt128/front/points"
    assert config.ros.task_manager_service == "/a2/task_manager/command"
    assert config.ros.localization_pose_topic == "/amcl_pose"
    assert config.ros.localization_pose_msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped"
    assert config.ros.pose_goal_status_topic == "/a2/nav2/status"
    assert config.ros.pointcloud_primary_stale_sec > 0.0
    assert config.ros.pointcloud_preview_max_points >= 20000
    assert config.stack.start_script.endswith("start_real_stack.sh")


def test_navigation_contract_uses_nav2_by_default():
    labels = {label for _, label, _ in NAVIGATION_NODES}
    patterns = {pattern for _, _, pattern in NAVIGATION_NODES}

    assert "AMCL localization" in labels
    assert "goal bridge" in labels
    assert "map server" in labels
    assert "planner server" in labels
    assert "controller server" in labels
    assert "bt navigator" in labels
    assert "amcl" in patterns
    assert "planner_server" in patterns
    assert "controller_server" in patterns
    assert "bt_navigator" in patterns
    assert "pcd_relocalizer_3d" in STACK_CLEANUP_PATTERNS
    assert "pose_goal_controller_3d" in STACK_CLEANUP_PATTERNS


def test_mapping_contract_accepts_slam_toolbox_and_native_fallbacks():
    mapping_patterns = {pattern for _, _, pattern in MAPPING_NODES}

    assert "slam_toolbox" in STACK_CLEANUP_PATTERNS
    assert "native_map_relay" in STACK_CLEANUP_PATTERNS
    assert "pointcloud_accumulator" in STACK_CLEANUP_PATTERNS
    assert ("jt128_dlio_map", "dlio_map_node") in mapping_patterns


def test_map_media_listing_contract_supports_image_pointcloud_linking():
    listing = MapMediaListing(
        map_id="demo_map",
        entries=[
            MapMediaEntry(
                kind="image",
                path="images/frame_001.png",
                name="frame_001.png",
                group="images",
                size_bytes=1024,
                artifact_kind=None,
                linked_pointcloud_path="PCD/frame_001.pcd",
                linked_image_path=None,
                link_source="metadata",
            ),
            MapMediaEntry(
                kind="pointcloud",
                path="PCD/frame_001.pcd",
                name="frame_001.pcd",
                group="PCD",
                size_bytes=2048,
                artifact_kind="pointcloud_snapshot_3d",
                linked_pointcloud_path=None,
                linked_image_path="images/frame_001.png",
                link_source="metadata",
            ),
        ],
    )

    assert listing.map_id == "demo_map"
    assert listing.entries[0].linked_pointcloud_path == "PCD/frame_001.pcd"
    assert listing.entries[1].linked_image_path == "images/frame_001.png"
    assert listing.entries[0].link_source == "metadata"


def test_task_route_and_virtual_obstacle_contracts_are_available():
    status = TaskRouteStatus(
        raw="mode=real;state=ready;ready=true;reason=idle;route_state=running;route_id=office_loop",
        ready=True,
        state="ready",
        reason="idle",
        current_mode="navigation",
        active_map="site_demo",
        route_state="running",
        route_id="office_loop",
        route_path="/tmp/office_loop.yaml",
        report_path="/tmp/report.md",
        fields={"route_state": "running"},
    )
    listing = VirtualObstacleListing(
        map_id="site_demo",
        obstacles=[
            VirtualObstacleZone(
                obstacle_id="dock_keepout",
                label="dock_keepout",
                kind="circle_keepout",
                x=1.0,
                y=2.0,
                radius=0.6,
            )
        ],
    )

    assert status.route_id == "office_loop"
    assert status.route_state == "running"
    assert listing.map_id == "site_demo"
    assert listing.obstacles[0].radius == 0.6
