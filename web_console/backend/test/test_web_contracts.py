from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import load_config
from backend.models import CameraFrame, DashboardSnapshot
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
    assert config.native_slam.enabled is True
    assert config.native_slam.request_topic == "/api/slam_operate/request"
    assert config.native_slam.response_topic == "/api/slam_operate/response"
    assert config.native_slam.response_timeout_sec >= 1.0


def test_navigation_contract_uses_amcl_not_manual_localization():
    labels = {label for _, label, _ in NAVIGATION_NODES}
    patterns = {pattern for _, _, pattern in NAVIGATION_NODES}

    assert "AMCL localization" in labels
    assert "amcl" in patterns
    assert "manual localization" not in labels
    assert "manual_localization_publisher" not in patterns
    assert "amcl" in STACK_CLEANUP_PATTERNS
    assert "task_manager.py" in STACK_CLEANUP_PATTERNS


def test_mapping_contract_accepts_slam_toolbox_and_native_fallbacks():
    mapping_patterns = {pattern for _, _, pattern in MAPPING_NODES}

    assert "slam_toolbox" in STACK_CLEANUP_PATTERNS
    assert "native_map_relay" in STACK_CLEANUP_PATTERNS
    assert ("slam_toolbox", "native_map_relay", "occupancy_mapper") in mapping_patterns
