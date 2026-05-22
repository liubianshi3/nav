from __future__ import annotations

import sys
import math
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import load_config
from backend.diagnostics import build_diagnostics, read_logs
from backend.direct_navigation import compute_direct_velocity_command
from backend.utils import extrapolate_pose2d_from_odom
from backend.models import (
    CameraFrame,
    DashboardSnapshot,
    GaitControlCommand,
    ManualVelocityCommand,
    MapMediaEntry,
    MapMediaListing,
    StackStatus,
    TaskRouteStatus,
    TextStatus,
    VirtualObstacleListing,
    VirtualObstacleZone,
    StartNavigationRequest,
)
from backend.navigation_rules import active_navigation_goal_conflict_reason
from backend.stack_control import (
    MAPPING_NODES,
    NAVIGATION_NODES,
    NAVIGATION_NODES_3D,
    NAVIGATION_MOTION_MODES,
    STACK_CLEANUP_PATTERNS,
    StackController,
)


def test_dashboard_snapshot_contains_camera_contract():
    snapshot = DashboardSnapshot()

    assert isinstance(snapshot.camera, CameraFrame)
    assert snapshot.camera.available is False
    assert snapshot.health.camera_received is False


def test_nav2_goal_can_retarget_while_previous_goal_is_active(monkeypatch):
    assert active_navigation_goal_conflict_reason(
        backend="nav2",
        has_active_action_goal=True,
        has_active_pose_goal=False,
    ) is None


def test_navigation_goal_block_reason_explains_missing_initialpose():
    from backend.navigation_rules import localization_goal_block_reason

    status = TextStatus(
        state="waiting_seed",
        ready=False,
        reason="send_initialpose",
        fields={"initial_guess_count": "0", "map_ready": "true", "odom_fresh": "true"},
    )

    assert localization_goal_block_reason(localization_ok=False, relocalization_status=status) == (
        "NDT 等待初始位姿，请先设置初始位姿"
    )


def test_diagnostics_explain_ndt_waiting_for_initialpose_not_generic_tf():
    snapshot = DashboardSnapshot()
    snapshot.pose.available = False
    snapshot.status.localization_ok = False
    snapshot.status.ndt_healthy = False
    snapshot.status.ndt_score = -1.0
    snapshot.status.relocalization_status = TextStatus(
        raw="state=waiting_seed;ready=false;reason=send_initialpose;score=-1.000;initial_guess_count=0",
        state="waiting_seed",
        ready=False,
        reason="send_initialpose",
        fields={"initial_guess_count": "0", "map_ready": "true", "odom_fresh": "true"},
    )

    diagnostics = build_diagnostics(snapshot, StackStatus(mode="navigation"))
    item = next(item for item in diagnostics.navigation if item.key == "localization")

    assert item.state == "error"
    assert item.reason == "NDT 等待初始位姿"
    assert item.suggestion == "在导航选择里点击地图当前位置，先设置初始位姿；NDT 收敛后会发布 map→odom TF"
    assert any("relocalization_state=waiting_seed" in evidence for evidence in item.evidence)


def test_nav2_3d_point_goal_does_not_force_final_heading_alignment():
    root = Path(__file__).resolve().parents[3]
    nav2_3d = yaml.safe_load((root / "src/a2_system/config/nav2_3d.yaml").read_text(encoding="utf-8"))

    goal_checker = nav2_3d["controller_server"]["ros__parameters"]["general_goal_checker"]

    assert goal_checker["yaw_goal_tolerance"] >= math.pi - 0.01


def test_3d_point_selection_preserves_current_robot_yaw():
    root = Path(__file__).resolve().parents[3]
    source = (root / "web_console/frontend/src/components/PointCloudCanvas3D.tsx").read_text(encoding="utf-8")

    selection_block = source[source.index("onSelectGoalRef.current({") : source.index("});", source.index("onSelectGoalRef.current({"))]

    assert "yaw: lastRobotPoseRef.current.yaw" in selection_block
    assert "yaw: 0" not in selection_block


def test_diagnostics_explain_unitree_agent_ipc_boundary_not_dds():
    snapshot = DashboardSnapshot()
    snapshot.map.loaded = True
    snapshot.map.width = 20
    snapshot.map.height = 20
    snapshot.map.resolution = 0.1
    snapshot.pose.available = True
    snapshot.pose.stale = False
    snapshot.pose.x = 0.5
    snapshot.pose.y = 0.5
    snapshot.status.localization_ok = True
    snapshot.status.ndt_healthy = True
    snapshot.status.safety_status = TextStatus(state="ready", ready=True, reason="clear")
    snapshot.status.control_status = TextStatus(state="timeout", ready=False, reason="ipc_unavailable")
    snapshot.status.sdk_status = TextStatus(state="waiting", ready=False, reason="waiting_for_agent_state")

    diagnostics = build_diagnostics(snapshot, StackStatus(mode="navigation"))
    control_item = next(item for item in diagnostics.navigation if item.key == "control_bridge")

    assert diagnostics.summary.severity == "error"
    assert "unitree_agent" in control_item.suggestion
    assert "UDS" in control_item.suggestion or "IPC" in control_item.suggestion
    assert "DDS" not in control_item.suggestion
    assert "SDK bridge" not in control_item.suggestion


def test_diagnostics_logs_classify_unitree_agent_ipc_as_control(tmp_path):
    log_file = tmp_path / "stack.log"
    log_file.write_text(
        "[unitree_agent] IPC unavailable while handling cmd_vel_safe\n"
        "[WARN] [1710000000.0] [a2_control_bridge]: ipc_unavailable on cmd_vel_safe\n"
        "[map_manager] saved map demo\n",
        encoding="utf-8",
    )

    entries = read_logs(str(log_file), source="control")

    assert len(entries) == 2
    assert entries[0].source == "unitree_agent"
    assert entries[0].category == "control"
    assert entries[1].source == "a2_control_bridge"


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
    assert config.ros.pointcloud_topic == "/jt128/dlio/map_points_preview"
    assert config.ros.pointcloud_fallback_topic == "/jt128/front/points_preview"
    assert config.ros.pointcloud_map_topics == [
        "/jt128/dlio/map_points_preview",
    ]
    assert all(topic.endswith("_preview") for topic in [config.ros.pointcloud_topic, config.ros.pointcloud_fallback_topic, *config.ros.pointcloud_map_topics])
    assert config.ros.task_manager_service == "/a2/task_manager/command"
    assert config.ros.localization_pose_topic == "/a2/relocalization/pose"  # 3D-first
    assert config.ros.localization_pose_msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped"
    assert config.ros.pose_goal_status_topic == "/a2/nav2/status"
    assert config.ros.pointcloud_primary_stale_sec > 0.0
    assert config.ros.pointcloud_preview_max_points >= 20000
    assert 1000 <= config.ros.websocket_pointcloud_max_points <= 20000
    assert config.health.websocket_pose_hz <= 10.0
    assert config.health.websocket_status_hz <= 5.0
    assert config.stack.start_script.endswith("start_jt128_3d_stack.sh")


def test_3d_config_uses_source_workspace_defaults():
    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")

    assert config.stack.workspace == "/home/unitree/ws/device-navigation"
    assert config.stack.map_root == "/home/unitree/ws/device-navigation/runtime/maps"
    assert config.stack.start_script == "/home/unitree/ws/device-navigation/src/a2_system/tools/start_jt128_3d_stack.sh"
    assert config.stack.stop_script == "/home/unitree/ws/device-navigation/src/a2_system/tools/stop_jt128_stack.sh"
    assert config.ros.pointcloud_topic == "/jt128/dlio/map_points_preview"
    assert config.ros.pointcloud_fallback_topic == "/jt128/front/points_preview"
    assert config.ros.pointcloud_map_topics == [
        "/jt128/dlio/map_points_preview",
    ]
    assert all(topic.endswith("_preview") for topic in [config.ros.pointcloud_topic, config.ros.pointcloud_fallback_topic, *config.ros.pointcloud_map_topics])
    assert config.ros.pointcloud_preview_max_points >= 60000
    assert config.ros.websocket_pointcloud_max_points >= 48000


def test_a2_workspace_env_overrides_3d_stack_paths(monkeypatch):
    monkeypatch.setenv("A2_WORKSPACE", "/tmp/a2_ws")
    monkeypatch.setenv("A2_NETWORK_INTERFACE", "enp3s0")

    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")

    assert config.stack.workspace == "/tmp/a2_ws"
    assert config.stack.map_root == "/tmp/a2_ws/runtime/maps"
    assert config.stack.start_script == "/tmp/a2_ws/src/a2_system/tools/start_jt128_3d_stack.sh"
    assert config.stack.stop_script == "/tmp/a2_ws/src/a2_system/tools/stop_jt128_stack.sh"
    assert config.stack.network_interface == "enp3s0"


def test_jt128_interface_env_overrides_stack_lidar_interface(monkeypatch):
    monkeypatch.setenv("A2_WORKSPACE", "/tmp/a2_ws")
    monkeypatch.setenv("A2_NETWORK_INTERFACE", "eth0")
    monkeypatch.setenv("A2_JT128_INTERFACE", "net1")

    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.yaml")
    command = StackController(config)._start_script_command("navigation", "a2_map")

    assert config.stack.network_interface == "net1"
    assert "--lidar-iface" in command
    assert command[command.index("--lidar-iface") + 1] == "net1"


def test_web_systemd_service_targets_current_a2_source_workspace():
    repo_root = Path(__file__).resolve().parents[3]
    unit_source = (repo_root / "web_console/systemd/a2-web-console.service").read_text(encoding="utf-8")
    suite_source = (repo_root / "src/a2_system/tools/start_web_console_suite.sh").read_text(encoding="utf-8")

    assert "/home/unitree/a2_system_ws" not in unit_source
    assert "/opt/a2_system_ws" not in unit_source
    assert "CONFIG_PATH=/home/unitree/ws/device-navigation/web_console/backend/config.3d.yaml" in unit_source
    assert "Environment=A2_WORKSPACE=/home/unitree/ws/device-navigation" in unit_source
    assert "CONFIG_PATH=${WORKSPACE}/web_console/backend/config.3d.yaml" in suite_source
    assert "WorkingDirectory=${WORKSPACE}/web_console" in suite_source


def test_a2_deploy_scripts_target_current_docker_workspace():
    repo_root = Path(__file__).resolve().parents[3]
    expected_workspace = "/home/unitree/ws/device-navigation"
    stale_workspace = "/home/unitree/a2_system_ws"
    paths = [
        repo_root / "scripts/deploy_to_a2.sh",
        repo_root / "scripts/start_a2_console.sh",
        repo_root / "scripts/start_a2_closed_loop.sh",
        repo_root / "scripts/start_a2_real1.sh",
        repo_root / "README.md",
        repo_root / "docker/README.md",
    ]

    for path in paths:
        source = path.read_text(encoding="utf-8")
        assert stale_workspace not in source
        assert expected_workspace in source


def test_docker_config_uses_raw_camera_when_compressed_topic_is_absent():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.yaml")

    assert config.camera.enabled is True
    assert config.camera.prefer_compressed is False
    assert config.ros.camera_image_topic == "/camera/image_raw"
    assert config.ros.pointcloud_topic == "/jt128/dlio/map_points_preview"
    assert config.ros.pointcloud_fallback_topic == "/jt128/front/points_preview"
    assert config.ros.pointcloud_map_topics == ["/jt128/dlio/map_points_preview"]
    assert all(topic.endswith("_preview") for topic in [config.ros.pointcloud_topic, config.ros.pointcloud_fallback_topic, *config.ros.pointcloud_map_topics])
    assert config.ros.odom_topic == "/odometry/local"
    assert config.navigation.backend == "nav2"
    assert config.navigation.goal_topic == "/a2/nav3/goal_pose"
    assert config.stack.command_timeout_sec >= 60.0


def test_docker_config_uses_jt128_3d_stack_and_keeps_manual_control():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.yaml")

    assert config.stack.start_script.endswith("start_jt128_3d_stack.sh")
    assert config.stack.stop_script.endswith("stop_jt128_stack.sh")
    assert config.stack.command_timeout_sec >= 60.0
    assert config.ros.localization_pose_topic == "/a2/relocalization/pose"
    assert config.ros.pointcloud_topic == "/jt128/dlio/map_points_preview"
    assert config.ros.pointcloud_fallback_topic == "/jt128/front/points_preview"
    assert config.ros.pointcloud_map_topics == ["/jt128/dlio/map_points_preview"]
    assert all(topic.endswith("_preview") for topic in [config.ros.pointcloud_topic, config.ros.pointcloud_fallback_topic, *config.ros.pointcloud_map_topics])
    assert config.ros.odom_topic == "/odometry/local"
    assert config.navigation.backend == "nav2"
    assert config.navigation.goal_topic == "/a2/nav3/goal_pose"
    assert config.manual_control.enabled is True
    assert config.manual_control.cmd_topic == "/cmd_vel_safe"


def test_jt128_navigation_starts_live_motion_not_dry_run():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.yaml")
    command = StackController(config)._start_script_command("navigation", "a2_map")

    assert "--enable-motion" in command
    assert "--live-motion" in command


def test_jt128_mapping_still_uses_lidar_interface_from_stack_config():
    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")
    command = StackController(config)._start_script_command("mapping", "")

    assert config.stack.network_interface == "net1"
    assert Path(command[0]).name == "start_jt128_3d_stack.sh"
    assert command[1:] == [
        "--mode",
        "mapping",
        "--lidar-iface",
        "net1",
        "--no-web",
    ]
    assert "eth0" not in command


def test_manual_control_standby_uses_unitree_sdk_interface_by_default(monkeypatch):
    monkeypatch.delenv("A2_SDK_INTERFACE", raising=False)
    monkeypatch.delenv("A2_CONTROL_INTERFACE", raising=False)
    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")
    controller = StackController(config)

    assert config.stack.network_interface == "net1"
    assert controller._standby_sdk_interface() == "eth0"
    assert controller._standby_control_interface() == "eth0"


def test_manual_control_standby_control_interface_follows_sdk_override(monkeypatch):
    monkeypatch.setenv("A2_SDK_INTERFACE", "eth1")
    monkeypatch.delenv("A2_CONTROL_INTERFACE", raising=False)
    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")
    controller = StackController(config)

    assert controller._standby_sdk_interface() == "eth1"
    assert controller._standby_control_interface() == "eth1"

    monkeypatch.setenv("A2_CONTROL_INTERFACE", "eth2")

    assert controller._standby_control_interface() == "eth2"


def test_default_navigation_requests_are_live_motion():
    request = StartNavigationRequest(map_id="real_map")

    assert request.motion_mode == "live_motion"
    assert "dry_run" not in NAVIGATION_MOTION_MODES


def test_a2_docker_defaults_start_standby_with_real_motion_available():
    repo_root = Path(__file__).resolve().parents[3]
    compose_source = (repo_root / "docker-compose.a2.yml").read_text(encoding="utf-8")
    entrypoint_source = (repo_root / "docker/entrypoint.sh").read_text(encoding="utf-8")
    a2_ros_env_source = (repo_root / "docker/a2_ros.env").read_text(encoding="utf-8")
    legacy_special_suffix = "".join(("z", "be"))

    assert legacy_special_suffix not in compose_source.lower()
    assert not (repo_root / "docker/docker-compose.a2.yml").exists()
    assert not (repo_root / f"web_console/backend/config.docker.{legacy_special_suffix}.yaml").exists()
    assert "A2_DOCKER_START_MODE: ${A2_DOCKER_START_MODE:-standby}" in compose_source
    assert "A2_ENABLE_MOTION: ${A2_ENABLE_MOTION:-true}" in compose_source
    assert "A2_LIVE_MOTION: ${A2_LIVE_MOTION:-true}" in compose_source
    assert "image: ${A2_DOCKER_IMAGE:-a2-nav:dev}" in compose_source
    assert "a2-nav:" in compose_source
    assert "container_name: ${A2_CONTAINER_NAME:-a2-nav}" in compose_source
    assert "platform:" not in compose_source
    assert "A2_REQUIRE_UNITREE_SDK: ${A2_REQUIRE_UNITREE_SDK:-OFF}" in compose_source
    assert "a2-unitree-agent:" in compose_source
    assert "- a2-unitree-agent" in compose_source
    assert "dockerfile: Dockerfile.unitree_agent" in compose_source
    assert "- /run/a2:/run/a2" in compose_source
    assert "A2_UNITREE_AGENT_SOCKET: /run/a2/unitree_agent.sock" in compose_source
    assert "env_file:" in compose_source
    assert "- ./docker/a2_ros.env" in compose_source
    assert "ROS_DOMAIN_ID=0" in a2_ros_env_source
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in a2_ros_env_source
    assert "a2-system-ws" not in compose_source
    assert "A2_ROS_INTERFACE=wlxe865d4707bf8" in a2_ros_env_source
    assert "A2_ROS_PEERS=" in a2_ros_env_source
    assert "A2_NETWORK_INTERFACE: ${A2_NETWORK_INTERFACE:-net1}" in compose_source
    assert "A2_JT128_INTERFACE: ${A2_JT128_INTERFACE:-net1}" in compose_source
    assert "A2_SDK_INTERFACE: ${A2_SDK_INTERFACE:-eth0}" in compose_source
    assert "A2_CONTROL_INTERFACE: ${A2_CONTROL_INTERFACE:-eth0}" in compose_source
    assert "A2_SDK_BRIDGE_AUTOSTART: ${A2_SDK_BRIDGE_AUTOSTART:-true}" in compose_source
    assert "A2_CONTROL_BRIDGE_AUTOSTART: ${A2_CONTROL_BRIDGE_AUTOSTART:-true}" in compose_source
    assert "A2_STANDBY_LIDAR_AUTOSTART: ${A2_STANDBY_LIDAR_AUTOSTART:-true}" in compose_source
    assert "A2_STANDBY_POINTCLOUD_PREVIEW_AUTOSTART: ${A2_STANDBY_POINTCLOUD_PREVIEW_AUTOSTART:-true}" in compose_source
    assert "A2_CONTROL_ALLOW_WITHOUT_MAP: ${A2_CONTROL_ALLOW_WITHOUT_MAP:-true}" in compose_source
    assert "A2_CONTROL_ALLOW_WITHOUT_LOCALIZATION: ${A2_CONTROL_ALLOW_WITHOUT_LOCALIZATION:-true}" in compose_source
    assert "${A2_HOST_MAP_ROOT:-/home/unitree/ws/device-navigation/runtime/maps}" in compose_source
    assert 'local enable_motion="${A2_ENABLE_MOTION:-true}"' in entrypoint_source
    assert 'local live_motion="${A2_LIVE_MOTION:-true}"' in entrypoint_source
    assert "start_standby_lidar_preview" in entrypoint_source
    assert 'A2_STANDBY_LIDAR_AUTOSTART:-true' in entrypoint_source
    assert 'ros2 launch a2_bringup jt128_driver.launch.py' in entrypoint_source
    assert 'ros2 run a2_system pointcloud_preview_node.py' in entrypoint_source
    assert 'start_standby_lidar_preview' in entrypoint_source.split("start_a2_stack")[0]
    assert "configure_cyclonedds_interface" in entrypoint_source
    assert 'local iface="${A2_ROS_INTERFACE:-}"' in entrypoint_source
    assert 'local peers="${A2_ROS_PEERS:-}"' in entrypoint_source
    assert 'export CYCLONEDDS_URI="<CycloneDDS xmlns=' in entrypoint_source
    assert '<Domain Id=\\"any\\">' in entrypoint_source
    assert '<AllowMulticast>spdp</AllowMulticast>' in entrypoint_source
    assert '<Peer Address=\\"${peer}\\" />' in entrypoint_source


def test_docker_ros_children_preserve_cyclonedds_uri_for_rviz_peers():
    repo_root = Path(__file__).resolve().parents[3]
    entrypoint_source = (repo_root / "docker/entrypoint.sh").read_text(encoding="utf-8")
    stack_source = (repo_root / "src/a2_system/tools/start_jt128_3d_stack.sh").read_text(encoding="utf-8")
    mapping_source = (repo_root / "src/a2_system/tools/start_jt128_dlio_mapping.sh").read_text(encoding="utf-8")

    assert "export_child_ros_env" in entrypoint_source
    assert "export_child_ros_env" in stack_source
    assert "export_child_ros_env" in mapping_source
    assert "export CYCLONEDDS_URI=" in entrypoint_source
    assert "export CYCLONEDDS_URI=" in stack_source
    assert "export CYCLONEDDS_URI=" in mapping_source


def test_unitree_bridge_nodes_use_cyclonedds_and_unitree_agent_ipc():
    repo_root = Path(__file__).resolve().parents[3]
    nav_launch = (repo_root / "src/a2_bringup/launch/jt128_3d_navigation.launch.py").read_text(encoding="utf-8")
    legacy_launch = (repo_root / "src/a2_bringup/launch/bringup.launch.py").read_text(encoding="utf-8")
    entrypoint_source = (repo_root / "docker/entrypoint.sh").read_text(encoding="utf-8")

    for source in (nav_launch, legacy_launch, entrypoint_source):
        assert "rmw_cyclonedds_cpp" in source
        assert "rmw_fastrtps_cpp" not in source
        assert "A2_UNITREE_RMW_IMPLEMENTATION" not in source
        assert "A2_CONTROL_BRIDGE_LD_PRELOAD" not in source
        assert "A2_SDK_BRIDGE_LD_PRELOAD" not in source
        assert "unitree_agent.sock" in source


def test_sim_config_uses_direct_cmd_vel_navigation_for_sim():
    config = load_config(Path(__file__).resolve().parents[1] / "config.sim.yaml")

    assert config.navigation.backend == "cmd_vel_direct"
    assert config.navigation.direct_cmd_topic == "/cmd_vel_safe"
    assert config.navigation.direct_goal_tolerance_m <= 0.25
    assert config.navigation.direct_max_linear_x <= 0.35
    assert config.navigation.direct_max_angular_z <= 0.8


def test_manual_control_contract_publishes_safe_cmd_vel():
    root = Path(__file__).resolve().parents[2]
    config = load_config(root / "backend/config.docker.yaml")
    command = ManualVelocityCommand(linear_x=0.2, angular_z=0.4)

    assert config.manual_control.enabled is True
    assert config.manual_control.cmd_topic == "/cmd_vel_safe"
    assert config.manual_control.max_linear_x <= 0.5
    assert config.manual_control.max_angular_z <= 1.0
    assert command.linear_x == 0.2
    assert command.angular_z == 0.4

    main_source = (root / "backend/main.py").read_text()
    manual_endpoint_source = main_source[
        main_source.index('    @app.post("/api/manual-control/cmd_vel")') :
        main_source.index('    @app.get("/api/manual-control/motion-authorization")')
    ]
    api_source = (root / "frontend/src/api.ts").read_text()
    app_source = (root / "frontend/src/App.tsx").read_text()
    controls_source = (root / "frontend/src/components/ControlSidebar.tsx").read_text()
    styles_source = (root / "frontend/src/styles.css").read_text()
    stack_source = (root / "backend/stack_control.py").read_text()
    grpc_source = (root / "backend/grpc_server.py").read_text()

    assert "/api/manual-control/cmd_vel" in main_source
    assert "stack_controller.ensure_manual_control_standby" in main_source
    assert "ensure_manual_motion_authorized" in main_source
    assert "stack_controller.ensure_manual_control_standby" not in manual_endpoint_source
    assert "ensure_manual_motion_authorized" not in manual_endpoint_source
    assert "publish_manual_velocity" in manual_endpoint_source
    assert "sendManualVelocityCommand" in api_source
    assert "ManualControlSection" in controls_source
    assert "snapshot.manual_control.enabled" in app_source
    assert "cmdTopic={snapshot.manual_control.cmd_topic}" in app_source
    assert "手动控制未启用" in app_source
    assert "onManualVelocityCommand" in app_source
    assert "toLocaleTimeString" in app_source
    assert "setLastManualControlMessage(`${stamp} ${result.message}`)" in app_source
    assert "cmdTopic" in controls_source
    assert "onMouseDown={(event) => handleMouseDown(key, event)}" in controls_source
    assert "onMouseUp={finishCommand}" in controls_source
    assert "onTouchStart={(event) => handleTouchStart(key, event)}" in controls_source
    assert "onTouchEnd={finishCommand}" in controls_source
    assert "shouldSkipClickReplay(key)" in controls_source
    assert "suppressClickRef" not in controls_source
    assert "onClick={() =>" in controls_source
    assert "按住方向键持续发布" in controls_source
    assert "StackModeChip" in controls_source
    assert 'toStackModeLabel(mode)' in controls_source
    assert 'return mode === "stopped" ? "standby" : mode' in controls_source
    assert ".task-chip-standby" in styles_source
    assert "manual-auth-grid" in controls_source
    assert "motionAlreadyAuthorized" in controls_source
    assert 'motionAlreadyAuthorized ? "已授权" : "启动运动授权"' in controls_source
    assert "disabled={buttonDisabled || motionAuthorizationBusy || motionAlreadyAuthorized}" in controls_source
    assert ".manual-auth-grid .status-value" in styles_source
    assert "overflow-wrap: anywhere" in styles_source
    assert "def ensure_manual_control_standby" in stack_source
    assert "start_standby_sdk_bridge" in stack_source
    assert "start_standby_control_bridge" in stack_source
    assert "def _standby_sdk_interface" in stack_source
    assert "def _standby_control_interface" in stack_source
    assert "def _manual_control_standby_mismatches" in stack_source
    assert "self.config.stack.network_interface" in stack_source
    assert "os.environ.get(\"A2_SDK_INTERFACE\") or \"eth0\"" in stack_source
    assert "ipc_socket_path:={shlex.quote(os.environ.get('A2_UNITREE_AGENT_SOCKET', '/run/a2/unitree_agent.sock'))}" in stack_source
    assert "socket_arg = f\"ipc_socket_path:={os.environ.get('A2_UNITREE_AGENT_SOCKET', '/run/a2/unitree_agent.sock')}\"" in stack_source
    assert "A2_CONTROL_ALLOW_WITHOUT_MAP" in stack_source
    assert "A2_CONTROL_ALLOW_WITHOUT_LOCALIZATION" in stack_source
    assert "ensure_manual_control_standby" in grpc_source
    assert "self.manual_control_publisher.publish(msg)" in (root / "backend/ros_bridge.py").read_text()
    assert "manual.publish_burst_count" not in (root / "backend/ros_bridge.py").read_text()


def test_real_3d_config_enables_manual_control_for_true_dog():
    config = load_config(Path(__file__).resolve().parents[1] / "config.3d.yaml")

    assert config.manual_control.enabled is True
    assert config.manual_control.cmd_topic == "/cmd_vel_safe"
    assert config.manual_control.max_linear_x <= 0.4
    assert config.manual_control.max_linear_y <= 0.25
    assert config.manual_control.max_angular_z <= 0.8
    assert config.manual_control.publish_burst_count == 1
    assert config.manual_control.publish_burst_interval_sec == 0.0


def test_gait_control_contract_publishes_unitree_sport_requests():
    root = Path(__file__).resolve().parents[2]
    config = load_config(root / "backend/config.docker.yaml")
    command = GaitControlCommand(gait_type=1, speed_level=1)

    assert config.gait_control.enabled is True
    assert config.gait_control.gait_type_topic == "/a2/control/gait_type"
    assert config.gait_control.speed_level_topic == "/a2/control/speed_level"
    assert config.gait_control.body_height_topic == "/a2/control/body_height"
    assert config.ros.control_status_topic == "/a2/control/status"
    assert command.gait_type == 1

    main_source = (root / "backend/main.py").read_text()
    api_source = (root / "frontend/src/api.ts").read_text()
    app_source = (root / "frontend/src/App.tsx").read_text()
    controls_source = (root / "frontend/src/components/ControlSidebar.tsx").read_text()

    assert "/api/gait-control" in main_source
    assert "sendGaitControlCommand" in api_source
    assert "GaitControlSection" in controls_source
    assert "onGaitControlCommand" in app_source


def test_initial_pose_button_renders_inline_feedback_in_navigation_drawer():
    root = Path(__file__).resolve().parents[2]
    app_source = (root / "frontend/src/App.tsx").read_text()
    controls_source = (root / "frontend/src/components/ControlSidebar.tsx").read_text()

    assert "initialPoseBusy" in app_source
    assert "lastInitialPoseMessage" in app_source
    assert "lastInitialPoseError" in app_source
    assert "正在设置初始位姿" in app_source
    assert "initialPoseBusy={initialPoseBusy}" in app_source
    assert "initialPoseMessage={lastInitialPoseMessage}" in app_source
    assert "initialPoseError={lastInitialPoseError}" in app_source

    assert "initialPoseBusy" in controls_source
    assert "initialPoseMessage" in controls_source
    assert "initialPoseError" in controls_source
    assert "initial-pose-feedback" in controls_source
    assert "disabled={initialPoseBusy || !selectedGoal || !canSetInitialPose}" in controls_source


def test_3d_robot_marker_does_not_snap_pose_to_pointcloud_surface():
    root = Path(__file__).resolve().parents[2]
    source = (root / "frontend/src/components/PointCloudCanvas3D.tsx").read_text(encoding="utf-8")

    assert "markerPositionFromRos(current, { x: markerPose.x, y: markerPose.y, z: 0 })" in source
    assert "markerPositionFromRos(current, { x: selectedGoal.x, y: selectedGoal.y, z: 0 })" in source
    assert "markerPositionFromRos(current, { x: activeGoal.x, y: activeGoal.y, z: 0 })" in source
    assert "function markerPositionFromRos(context: SceneContext | null, point: { x: number; y: number; z: number })" in source
    assert "snapToSurface" not in source


def test_3d_yellow_robot_marker_follows_pose_without_static_origin_marker():
    root = Path(__file__).resolve().parents[2]
    source = (root / "frontend/src/components/PointCloudCanvas3D.tsx").read_text(encoding="utf-8")

    assert "lastRobotPoseRef" in source
    assert "useRef<{ x: number; y: number; yaw: number }>({ x: 0, y: 0, yaw: 0 })" in source
    assert "lastRobotPoseRef.current = {" in source
    assert "const markerPose = lastRobotPoseRef.current;" in source
    assert "updateMarker(\n      current?.robotMarker ?? null,\n      markerPositionFromRos(current, { x: markerPose.x, y: markerPose.y, z: 0 })," in source
    assert "scene.add(robotMarker, selectedGoalMarker, activeGoalMarker);" in source
    assert "originMarker" not in source
    assert "createOriginMarker" not in source


def test_3d_double_click_can_select_ground_plane_without_point_hit():
    root = Path(__file__).resolve().parents[2]
    source = (root / "frontend/src/components/PointCloudCanvas3D.tsx").read_text(encoding="utf-8")

    assert "const planeHit = pickGroundPlanePoint(raycaster);" in source
    assert "function pickGroundPlanePoint(raycaster: THREE.Raycaster): THREE.Vector3 | null" in source


def test_mapping_3d_view_ignores_selected_saved_pointcloud():
    root = Path(__file__).resolve().parents[2]
    source = (root / "frontend/src/App.tsx").read_text(encoding="utf-8")

    assert 'const viewerSelectedMap = stack?.mode === "mapping" ? null : selectedMap;' in source
    assert 'const viewerSelectedPointcloudPath = stack?.mode === "mapping" ? null : selectedPointcloudPath;' in source
    assert "selectedMap={viewerSelectedMap}" in source
    assert "selectedPointcloudPath={viewerSelectedPointcloudPath}" in source


def test_direct_navigation_command_turns_then_drives_to_goal():
    command = compute_direct_velocity_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=1.57,
        goal_x=1.0,
        goal_y=0.0,
        goal_yaw=0.0,
        max_linear_x=0.3,
        max_angular_z=0.6,
        slow_radius_m=0.6,
        heading_deadband_rad=0.25,
        goal_tolerance_m=0.15,
        yaw_tolerance_rad=0.25,
    )

    assert command.reached is False
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0

    command = compute_direct_velocity_command(
        current_x=0.0,
        current_y=0.0,
        current_yaw=0.0,
        goal_x=1.0,
        goal_y=0.0,
        goal_yaw=0.0,
        max_linear_x=0.3,
        max_angular_z=0.6,
        slow_radius_m=0.6,
        heading_deadband_rad=0.25,
        goal_tolerance_m=0.15,
        yaw_tolerance_rad=0.25,
    )

    assert command.reached is False
    assert command.linear_x > 0.0
    assert abs(command.angular_z) < 0.05

    command = compute_direct_velocity_command(
        current_x=1.0,
        current_y=0.0,
        current_yaw=0.0,
        goal_x=1.03,
        goal_y=0.0,
        goal_yaw=0.05,
        max_linear_x=0.3,
        max_angular_z=0.6,
        slow_radius_m=0.6,
        heading_deadband_rad=0.25,
        goal_tolerance_m=0.15,
        yaw_tolerance_rad=0.25,
    )

    assert command.reached is True
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_pose_fallback_extrapolates_map_pose_from_local_odom_delta():
    x, y, yaw = extrapolate_pose2d_from_odom(
        anchor_pose=(10.0, 5.0, 1.0),
        anchor_odom=(1.0, 2.0, 0.25),
        current_odom=(2.0, 2.0, 0.35),
    )

    assert math.isclose(x, 10.7316888689, rel_tol=1e-6)
    assert math.isclose(y, 5.6816387600, rel_tol=1e-6)
    assert math.isclose(yaw, 1.1, rel_tol=1e-6)


def test_navigation_contract_uses_nav2_by_default():
    labels = {label for _, label, _ in NAVIGATION_NODES}
    patterns = {pattern for _, _, pattern in NAVIGATION_NODES}

    assert "3D NDT localization" in labels  # legacy "AMCL localization" removed
    assert "goal bridge" in labels
    assert "map server" in labels
    assert "planner server" in labels
    assert "controller server" in labels
    assert "bt navigator" in labels
    # expand tuple patterns for membership checks
    flat_patterns = set()
    for p in patterns:
        if isinstance(p, tuple):
            flat_patterns.update(p)
        else:
            flat_patterns.add(p)
    assert ("ndt_adapter" in flat_patterns or "localization_gate" in flat_patterns)  # 3D-first, legacy "amcl" removed
    assert "planner_server" in patterns
    assert "controller_server" in patterns
    assert "bt_navigator" in patterns
    assert "pcd_relocalizer_3d" in STACK_CLEANUP_PATTERNS
    assert "pose_goal_controller_3d" in STACK_CLEANUP_PATTERNS


def test_3d_navigation_contract_uses_nav2_3d_not_legacy_pose_controller():
    labels = {label for _, label, _ in NAVIGATION_NODES_3D}
    patterns = {pattern for _, _, pattern in NAVIGATION_NODES_3D}

    assert "SmacPlanner2D planner server" in labels
    assert "DWB local controller server" in labels
    assert "Nav2 3D BT navigator" in labels
    assert "3D pose goal controller" not in labels
    assert "planner_server" in patterns
    assert "controller_server" in patterns
    assert "bt_navigator" in patterns
    assert "pose_goal_controller_3d" not in patterns


def test_3d_navigation_algorithm_contract_is_smac2d_plus_dwb():
    root = Path(__file__).resolve().parents[3]
    nav2_3d = yaml.safe_load((root / "src/a2_system/config/nav2_3d.yaml").read_text(encoding="utf-8"))
    planner = nav2_3d["planner_server"]["ros__parameters"]["GridBased"]
    controller = nav2_3d["controller_server"]["ros__parameters"]["FollowPath"]

    assert planner["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert controller["plugin"] == "dwb_core::DWBLocalPlanner"


def test_live_validation_collision_monitor_does_not_slow_navigation_commands():
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load(
        (root / "src/a2_system/config/collision_monitor_live_validation.yaml").read_text(encoding="utf-8")
    )
    polygon_slow = config["collision_monitor"]["ros__parameters"]["PolygonSlow"]

    assert polygon_slow["action_type"] == "slowdown"
    assert polygon_slow["slowdown_ratio"] == 1.0


def test_a2_live_navigation_speed_defaults_match_field_validated_limits():
    root = Path(__file__).resolve().parents[3]
    motion_limits = yaml.safe_load((root / "src/a2_system/config/motion_limits.yaml").read_text(encoding="utf-8"))
    nav2_3d = yaml.safe_load((root / "src/a2_system/config/nav2_3d.yaml").read_text(encoding="utf-8"))
    compose_source = (root / "docker-compose.a2.yml").read_text(encoding="utf-8")

    control = motion_limits["a2_control_bridge"]["ros__parameters"]
    controller = nav2_3d["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav2_3d["velocity_smoother"]["ros__parameters"]

    assert control["max_linear_x"] == 1.0
    assert control["max_linear_y"] == 0.45
    assert control["max_yaw_rate"] == 1.2
    assert control["cmd_timeout_sec"] == 1.0

    assert controller["max_vel_x"] == 0.9
    assert controller["max_vel_y"] == 0.45
    assert controller["max_vel_theta"] == 1.5
    assert controller["max_speed_xy"] == 0.9
    assert smoother["max_velocity"] == [0.9, 0.45, 1.5]

    assert "A2_CONTROL_MAX_LINEAR_X: ${A2_CONTROL_MAX_LINEAR_X:-1.0}" in compose_source
    assert "A2_CONTROL_MAX_LINEAR_Y: ${A2_CONTROL_MAX_LINEAR_Y:-0.45}" in compose_source
    assert "A2_CONTROL_MAX_YAW_RATE: ${A2_CONTROL_MAX_YAW_RATE:-1.2}" in compose_source
    assert "A2_CONTROL_CMD_TIMEOUT_SEC: ${A2_CONTROL_CMD_TIMEOUT_SEC:-1.0}" in compose_source


def test_a2_3d_nav2_controller_budget_matches_live_robot_cpu_limits():
    root = Path(__file__).resolve().parents[3]
    nav2_3d = yaml.safe_load((root / "src/a2_system/config/nav2_3d.yaml").read_text(encoding="utf-8"))
    ground_seg = yaml.safe_load((root / "src/a2_system/config/ground_segmentation.yaml").read_text(encoding="utf-8"))
    ndt = yaml.safe_load((root / "src/a2_system/config/ndt_scan_matcher_a2.yaml").read_text(encoding="utf-8"))

    controller_params = nav2_3d["controller_server"]["ros__parameters"]
    controller = controller_params["FollowPath"]
    smoother = nav2_3d["velocity_smoother"]["ros__parameters"]
    local_costmap = nav2_3d["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle_layer = local_costmap["obstacle_layer"]

    assert controller_params["controller_frequency"] <= 10.0
    assert smoother["smoothing_frequency"] <= 10.0
    assert controller["vx_samples"] * controller["vy_samples"] * controller["vtheta_samples"] <= 600
    assert controller["sim_time"] <= 1.2
    assert local_costmap["width"] <= 6
    assert local_costmap["height"] <= 6
    assert local_costmap["resolution"] >= 0.05
    assert obstacle_layer["enabled"] is True
    assert obstacle_layer["observation_sources"] == "obstacle_cloud"
    assert obstacle_layer["obstacle_cloud"]["obstacle_min_range"] >= 0.85
    assert ground_seg["ground_segmentation"]["ros__parameters"]["process_every_n"] >= 2
    assert ndt["/**"]["ros__parameters"]["ndt"]["max_iterations"] <= 40


def test_strict_collision_monitor_uses_forward_stop_without_slowdown_throttle():
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((root / "src/a2_system/config/collision_monitor.yaml").read_text(encoding="utf-8"))
    params = config["collision_monitor"]["ros__parameters"]
    polygon_stop = params["PolygonStop"]
    polygon_slow = params["PolygonSlow"]

    stop_xs = polygon_stop["points"][0::2]
    slow_xs = polygon_slow["points"][0::2]
    assert min(stop_xs) >= 0.75
    assert min(slow_xs) >= 0.75
    assert polygon_stop["max_points"] >= 20
    assert polygon_slow["action_type"] == "slowdown"
    assert polygon_slow["slowdown_ratio"] == 1.0


def test_jt128_hesai_config_matches_sdk2_schema():
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load(
        (root / "src/a2_system/config/jt128_front_hesai.yaml").read_text(encoding="utf-8")
    )
    driver = config["lidar"][0]["driver"]
    udp = driver["lidar_udp_type"]
    ros = config["lidar"][0]["ros"]

    assert "device_ip_address" not in driver
    assert udp["device_ip_address"] == "192.168.124.20"
    assert udp["udp_port"] == 2368
    assert udp["use_ptc_connected"] is True
    assert udp["host_ip_address"] == ""
    assert ros["ros_send_point_cloud_topic"] == "/jt128/front/points"


def test_jt128_startup_does_not_treat_socket_open_success_as_failure():
    root = Path(__file__).resolve().parents[3]
    script = (root / "src/a2_system/tools/start_jt128_dlio_mapping.sh").read_text(encoding="utf-8")

    assert "SocketSource::Open" not in script
    assert "bind failed|open udp source failed|\\\\[FATAL\\\\]" in script
    assert "reset_ros2_daemon()" in script
    assert "ros2 daemon stop" in script
    assert "wait_topic_message /jt128/front/points 12" in script


def test_jt128_dlio_mapping_does_not_default_to_graph_pid_overlay():
    root = Path(__file__).resolve().parents[3]
    script = (root / "src/a2_system/tools/start_jt128_dlio_mapping.sh").read_text(encoding="utf-8")

    assert 'GRAPH_PID_WS="${A2_GRAPH_PID_WS:-}"' in script
    assert 'A2_GRAPH_PID_WS:-$HOME/graph_pid_ws' not in script
    assert 'set A2_GRAPH_PID_WS explicitly' in script


def test_octomap_mapping_node_publishes_dlio_tf_chain():
    root = Path(__file__).resolve().parents[3]
    node = (root / "src/a2_system/scripts/octomap_mapping_node.py").read_text(encoding="utf-8")

    assert "TransformBroadcaster" in node
    assert "odom_to_base" in node
    assert "base_to_lidar" in node
    assert "jt128_front_link" in node


def test_docker_image_makes_a2_system_python_scripts_executable():
    root = Path(__file__).resolve().parents[3]
    dockerfile = (root / "Dockerfile").read_text(encoding="utf-8")

    assert "chmod +x src/a2_system/scripts/*.py" in dockerfile


def test_dlio_mapping_launch_uses_si_imu_converter():
    root = Path(__file__).resolve().parents[3]
    launch = (root / "src/a2_bringup/launch/dlio_mapping.launch.py").read_text(encoding="utf-8")

    assert "imu_to_si_converter.py" in launch
    assert "/jt128/front/imu_si" in launch
    assert "start_imu_si_converter" in launch
    assert 'DeclareLaunchArgument("imu_angular_velocity_scale", default_value="0.017453292519943295")' in launch
    assert 'os.environ.get("A2_WORKSPACE"' in launch


def test_jt128_lidar_and_imu_axes_are_configured_separately():
    root = Path(__file__).resolve().parents[3]
    dlio_config = (root / "src/a2_system/config/dlio_jt128.yaml").read_text(encoding="utf-8")
    extrinsics_config = (root / "src/a2_system/config/jt128_extrinsics.yaml").read_text(encoding="utf-8")

    assert "extrinsics/baselink2imu/R: [0.0, 0.0, 1.0," in dlio_config
    assert "extrinsics/baselink2lidar/R: [0.0, 0.0, 1.0," in dlio_config
    assert "0.0, -1.0, 0.0]" in dlio_config
    assert "The JT128 point cloud axes differ from the internal IMU axes" in dlio_config
    assert "rotation_matrix: [0.0, 0.0, 1.0," in extrinsics_config
    assert "0.0, -1.0, 0.0]" in extrinsics_config
    assert "jt128_internal_imu" in extrinsics_config


def test_jt128_dlio_watchdog_does_not_stop_mapping_on_single_speed_spike():
    root = Path(__file__).resolve().parents[3]
    watchdog = (root / "src/a2_system/scripts/jt128_dlio_watchdog.py").read_text(encoding="utf-8")
    launch = (root / "src/a2_bringup/launch/dlio_mapping.launch.py").read_text(encoding="utf-8")

    assert "fault_sample_count" in watchdog
    assert "self.pending_fault_count" in watchdog
    assert "state=\"suspect\"" in watchdog
    assert "self.pending_fault_count = 0" in watchdog
    assert '"stop_on_fault": False' in launch


def test_web_bridge_falls_back_to_local_odom_pose_when_relocalization_missing():
    root = Path(__file__).resolve().parents[3]
    bridge = (root / "web_console/backend/ros_bridge.py").read_text(encoding="utf-8")

    assert "def _should_use_odom_pose_fallback" in bridge
    assert "def _pose_from_odom" in bridge
    assert "source=self.config.ros.odom_topic" in bridge


def test_websocket_odom_updates_are_rate_limited_to_keep_pointcloud_responsive():
    root = Path(__file__).resolve().parents[3]
    bridge = (root / "web_console/backend/ros_bridge.py").read_text(encoding="utf-8")
    config = (root / "web_console/backend/config.py").read_text(encoding="utf-8")

    assert "websocket_pose_hz" in config
    assert "websocket_status_hz" in config
    assert "websocket_battery_hz" in config
    assert "def _publish_rate_limited" in bridge
    assert "def _publish_status" in bridge
    assert "def _publish_battery" in bridge
    assert 'key="status"' in bridge
    assert 'key="battery"' in bridge
    assert 'key="odom_pose"' in bridge
    odom_handler = bridge[bridge.index("def _on_odom") : bridge.index("def _on_tf")]
    status_handlers = bridge[bridge.index("def _on_real_report") : bridge.index("def _on_pose_goal_status")]
    battery_handler = bridge[bridge.index("def _on_battery") : bridge.index("def _on_scan_mission_status")]
    assert 'self._publish("health", self.get_health_dict())' not in odom_handler
    assert 'self._publish("status", dump_model(self.status))' not in status_handlers
    assert 'self._publish("battery", dump_model(self.battery))' not in battery_handler


def test_websocket_pointcloud_uses_lightweight_preview():
    root = Path(__file__).resolve().parents[3]
    bridge = (root / "web_console/backend/ros_bridge.py").read_text(encoding="utf-8")
    main = (root / "web_console/backend/main.py").read_text(encoding="utf-8")
    config = (root / "web_console/backend/config.py").read_text(encoding="utf-8")

    assert "websocket_pointcloud_max_points" in config
    assert "def _websocket_pointcloud_snapshot" in bridge
    assert "def _preview_sample_indices" in bridge
    assert "def _coprime_preview_stride" in bridge
    assert "round(float(point[0]), 3)" in bridge
    assert "_preview_sample_indices(len(snapshot.points), max_points)" in bridge
    assert "_preview_sample_indices(total_points, max_points)" in bridge
    assert "def _is_web_visualization_pointcloud_topic" in bridge
    assert "not _is_web_visualization_pointcloud_topic(topic)" in bridge
    assert "pointcloud_qos = QoSProfile" in bridge
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in bridge
    assert 'self._publish("pointcloud", dump_model(self._websocket_pointcloud_snapshot(self.pointcloud_snapshot)))' in bridge
    assert "snapshot.pointcloud = node._websocket_pointcloud_snapshot(snapshot.pointcloud)" in main


def test_frontend_warns_when_websocket_disconnected_without_snapshot_polling():
    root = Path(__file__).resolve().parents[3]
    app = (root / "web_console/frontend/src/App.tsx").read_text(encoding="utf-8")
    socket_hook = (root / "web_console/frontend/src/hooks/useBackendSocket.ts").read_text(encoding="utf-8")

    disconnected_handler = app[app.index("!websocketConnected") : app.index("legacy-function-button")]
    assert "正在自动重连" in disconnected_handler
    assert "fetchSnapshot()" not in disconnected_handler
    assert "window.setTimeout(connect, 1500)" in socket_hook


def test_mapping_mode_can_open_3d_view_before_projected_map_exists():
    root = Path(__file__).resolve().parents[3]
    app = (root / "web_console/frontend/src/App.tsx").read_text(encoding="utf-8")

    has_3d_viewer_data = app[app.index("const has3DViewerData") : app.index("const directNavigationBackend")]
    assert 'stack?.mode === "mapping"' in has_3d_viewer_data


def test_frontend_map_signature_tracks_projected_map_yaml():
    root = Path(__file__).resolve().parents[3]
    app = (root / "web_console/frontend/src/App.tsx").read_text(encoding="utf-8")

    maps_signature = app[app.index("function mapsSignature") : app.index("function setMapsIfChanged")]

    assert "map.map_yaml ??" in maps_signature


def test_initial_pose_readiness_waits_for_localization_pose_not_odom_fallback():
    root = Path(__file__).resolve().parents[3]
    bridge = (root / "web_console/backend/ros_bridge.py").read_text(encoding="utf-8")

    assert "_last_localization_pose_stamp" in bridge
    assert "_localization_pose_update_seen" in bridge
    assert "_initial_pose_ready(previous_localization_pose_stamp" in bridge
    assert "_pose_from_anchored_odom" in bridge


def test_jt128_navigation_uses_relaxed_ndt_correction_limits_for_real_maps():
    root = Path(__file__).resolve().parents[3]
    launch = (root / "src/a2_bringup/launch/jt128_3d_navigation.launch.py").read_text(encoding="utf-8")

    assert "ndt_max_map_to_odom_translation_step" in launch
    assert "default_value=\"5.0\"" in launch
    assert "max_map_to_odom_translation_step" in launch
    assert "max_map_to_odom_rotation_step_deg" in launch


def test_jt128_navigation_lifecycle_manages_collision_monitor():
    root = Path(__file__).resolve().parents[3]
    launch = (root / "src/a2_bringup/launch/jt128_3d_navigation.launch.py").read_text(encoding="utf-8")
    nav2_3d = (root / "src/a2_system/config/nav2_3d.yaml").read_text(encoding="utf-8")

    assert "collision_monitor lifecycle: managed by lifecycle_manager_navigation" in launch
    assert "condition=UnlessCondition(LaunchConfiguration(\"enable_nav2_3d\"))" in launch
    assert "lifecycle_manager_navigation:" in nav2_3d
    assert "collision_monitor" in nav2_3d


def test_stack_process_patterns_include_3d_ndt_navigation_nodes():
    root = Path(__file__).resolve().parents[3]
    stack_control = (root / "web_console/backend/stack_control.py").read_text(encoding="utf-8")

    assert "autoware_ndt_scan_matcher_node" in stack_control
    assert "ndt_adapter_node" in stack_control


def test_mapping_contract_accepts_slam_toolbox_and_native_fallbacks():
    mapping_patterns = {pattern for _, _, pattern in MAPPING_NODES}

    assert "slam_toolbox" in STACK_CLEANUP_PATTERNS  # kept in cleanup for legacy process kill safety
    assert "native_map_relay" in STACK_CLEANUP_PATTERNS
    assert "pointcloud_accumulator" in STACK_CLEANUP_PATTERNS
    assert ("jt128_dlio_map", "dlio_map_node") in mapping_patterns
    assert "octomap_mapping_node.py" in mapping_patterns
    assert "octomap_server_node" in mapping_patterns
    assert "octomap_mapping_node.py" in STACK_CLEANUP_PATTERNS
    assert "octomap_server_node" in STACK_CLEANUP_PATTERNS


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
