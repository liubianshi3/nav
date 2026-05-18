from __future__ import annotations

import sys
import math
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.config import load_config
from backend.direct_navigation import compute_direct_velocity_command
from backend.utils import extrapolate_pose2d_from_odom
from backend.models import (
    CameraFrame,
    DashboardSnapshot,
    GaitControlCommand,
    ManualVelocityCommand,
    MapMediaEntry,
    MapMediaListing,
    TaskRouteStatus,
    VirtualObstacleListing,
    VirtualObstacleZone,
)
from backend.stack_control import (
    MAPPING_NODES,
    NAVIGATION_NODES,
    NAVIGATION_NODES_3D,
    STACK_CLEANUP_PATTERNS,
    StackController,
)


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
    assert config.ros.localization_pose_topic == "/a2/relocalization/pose"  # 3D-first
    assert config.ros.localization_pose_msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped"
    assert config.ros.pose_goal_status_topic == "/a2/nav2/status"
    assert config.ros.pointcloud_primary_stale_sec > 0.0
    assert config.ros.pointcloud_preview_max_points >= 20000
    assert config.stack.start_script.endswith("start_jt128_3d_stack.sh")


def test_docker_config_uses_raw_camera_when_compressed_topic_is_absent():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.yaml")

    assert config.camera.enabled is True
    assert config.camera.prefer_compressed is False
    assert config.ros.camera_image_topic == "/camera/image_raw"
    assert config.navigation.backend == "nav2"


def test_zbe_docker_config_uses_jt128_3d_stack_and_keeps_manual_control():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.zbe.yaml")

    assert config.stack.start_script.endswith("start_jt128_3d_stack.sh")
    assert config.stack.stop_script.endswith("stop_jt128_stack.sh")
    assert config.stack.command_timeout_sec >= 60.0
    assert config.ros.localization_pose_topic == "/a2/relocalization/pose"
    assert config.ros.pointcloud_topic == "/jt128/front/points"
    assert config.ros.pointcloud_fallback_topic == "/a2/map/pointcloud_3d"
    assert config.ros.odom_topic == "/odometry/local"
    assert config.navigation.backend == "nav2"
    assert config.navigation.goal_topic == "/a2/nav3/goal_pose"
    assert config.manual_control.enabled is True
    assert config.manual_control.cmd_topic == "/cmd_vel_safe"


def test_zbe_jt128_navigation_starts_live_motion_not_dry_run():
    config = load_config(Path(__file__).resolve().parents[1] / "config.docker.zbe.yaml")
    command = StackController(config)._start_script_command("navigation", "zbe_map")

    assert "--enable-motion" in command
    assert "--live-motion" in command


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
    api_source = (root / "frontend/src/api.ts").read_text()
    app_source = (root / "frontend/src/App.tsx").read_text()
    controls_source = (root / "frontend/src/components/ControlSidebar.tsx").read_text()

    assert "/api/manual-control/cmd_vel" in main_source
    assert "sendManualVelocityCommand" in api_source
    assert "ManualControlSection" in controls_source
    assert "onManualVelocityCommand" in app_source


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


def test_web_bridge_falls_back_to_local_odom_pose_when_relocalization_missing():
    root = Path(__file__).resolve().parents[3]
    bridge = (root / "web_console/backend/ros_bridge.py").read_text(encoding="utf-8")

    assert "def _should_use_odom_pose_fallback" in bridge
    assert "def _pose_from_odom" in bridge
    assert "source=self.config.ros.odom_topic" in bridge


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

    assert "lifecycle_manager_collision_monitor" in launch
    assert '"node_names": ["collision_monitor"]' in launch


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
