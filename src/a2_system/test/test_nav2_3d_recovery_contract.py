from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _xml_tags(path: Path) -> set[str]:
    root = ET.parse(path).getroot()
    return {node.tag for node in root.iter()}


def test_real_a2_navigation_bt_recovery_is_non_motion_only():
    for filename in ("a2_navigate_3d.xml", "a2_navigate_through_poses_3d.xml"):
        tags = _xml_tags(CONFIG_DIR / filename)

        assert "Spin" not in tags
        assert "DriveOnHeading" not in tags
        assert "BackUp" not in tags
        assert "Wait" in tags
        assert "ClearEntireCostmap" in tags


def test_nav2_3d_behavior_server_does_not_load_motion_recovery_plugins():
    text = (CONFIG_DIR / "nav2_3d.yaml").read_text()

    assert re.search(r'behavior_plugins:\s*\["wait"\]', text)
    assert "nav2_behaviors/Spin" not in text
    assert "nav2_behaviors/DriveOnHeading" not in text
    assert "nav2_spin_action_bt_node" not in text
    assert "nav2_drive_on_heading_bt_node" not in text


def test_real_a2_source_launch_uses_cyclonedds_with_unitree_bridge_isolation():
    repo_root = Path(__file__).resolve().parents[3]
    launch_text = (
        repo_root / "src" / "a2_bringup" / "launch" / "jt128_3d_navigation.launch.py"
    ).read_text()
    start_text = (
        repo_root / "src" / "a2_system" / "tools" / "start_jt128_3d_stack.sh"
    ).read_text()
    dlio_start_text = (
        repo_root / "src" / "a2_system" / "tools" / "start_jt128_dlio_mapping.sh"
    ).read_text()

    assert '"rmw_fastrtps_cpp"' in launch_text
    assert "PreconditionNotMetError" in launch_text
    assert "ROS_RMW_IMPLEMENTATION" in start_text
    assert "rmw_cyclonedds_cpp" in start_text
    assert "unset FASTDDS_BUILTIN_TRANSPORTS" in start_text
    assert "ROS_RMW_IMPLEMENTATION" in dlio_start_text
    assert "rmw_cyclonedds_cpp" in dlio_start_text
    assert "unset FASTDDS_BUILTIN_TRANSPORTS" in dlio_start_text
