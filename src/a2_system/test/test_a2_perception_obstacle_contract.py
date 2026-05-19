from __future__ import annotations

from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text()


def _yaml_scalar(text: str, key: str) -> str:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([^#\n]+)", text, re.MULTILINE)
    assert match, f"missing YAML scalar {key}"
    return match.group(1).strip().strip("\"'")


def test_ground_segmentation_classifies_in_robot_frame_and_filters_a2_near_body():
    config = _read("src/a2_ground_segmentation_cpp/config/ground_segmentation_cpp.yaml")

    assert _yaml_scalar(config, "target_frame") == "map"
    assert _yaml_scalar(config, "classification_frame") == "base_link"
    assert _yaml_scalar(config, "classification_ground_plane_enabled") == "true"
    assert float(_yaml_scalar(config, "classification_ground_plane_a")) < -0.05
    assert abs(float(_yaml_scalar(config, "classification_ground_plane_b"))) < 0.20
    assert float(_yaml_scalar(config, "classification_ground_plane_c")) > 0.30
    assert _yaml_scalar(config, "self_filter_frame") == "base_link"
    assert float(_yaml_scalar(config, "self_filter_min_x")) <= -0.70
    assert float(_yaml_scalar(config, "self_filter_max_x")) >= 0.95
    assert float(_yaml_scalar(config, "self_filter_min_y")) <= -0.55
    assert float(_yaml_scalar(config, "self_filter_max_y")) >= 0.55
    assert float(_yaml_scalar(config, "self_filter_min_z")) <= -0.30
    assert float(_yaml_scalar(config, "self_filter_max_z")) >= 0.90


def test_ground_segmentation_node_keeps_classification_frame_separate_from_output_frame():
    node = _read("src/a2_ground_segmentation_cpp/src/ground_segmentation_cpp_node.cpp")

    assert 'declare_parameter<std::string>("classification_frame", "base_link")' in node
    assert 'declare_parameter<double>("classification_z_offset_m", 0.0)' in node
    assert "classification_z_offset_m_" in node
    assert '"classification_ground_plane_enabled", false' in node
    assert "classification_ground_plane_a_" in node
    assert "classification_from_source" in node
    assert "target_from_classification" in node
    assert "segmenter_->classify(classification_xyz)" in node


def test_traversability_obstacle_cloud_self_filters_before_nav2_obstacle_layer():
    script = _read("src/a2_system/scripts/traversability_to_obstacle_cloud.py")
    launch = _read("src/a2_bringup/launch/nav2_3d.launch.py")

    assert "self_filter_enabled" in script
    assert "_apply_self_filter" in script
    assert '"self_filter_enabled": True' in launch
    assert '"self_filter_max_x": 0.95' in launch
    assert '"self_filter_max_z": 0.90' in launch


def test_map_projection_clears_short_goal_corridor_around_current_pose():
    config = _read("src/a2_system/config/map_manager.yaml")

    radius = float(_yaml_scalar(config, "octomap_clear_current_pose_radius"))
    assert radius >= 1.20
