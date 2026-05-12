import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from a2_bringup.runtime_mode import as_bool, normalize_runtime_mode, use_sim_time_for_mode


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _resolve_config_path(a2_system_share, override_path, default_path, candidate_paths):
    override_path = (override_path or "").strip()
    if override_path:
        return override_path
    for candidate in candidate_paths:
        if candidate and os.path.exists(candidate):
            return candidate
    if default_path and os.path.exists(default_path):
        return default_path
    return default_path


def _real_lidar_consumer_topic(real_lidar_config_path, a2_system_share):
    params = _load_yaml(real_lidar_config_path).get("real_lidar", {}).get("ros__parameters", {})
    profile = params.get("profile", "")
    driver_mode = params.get("driver_mode", "")
    if profile == "unitree_native_fused" or driver_mode == "external_pointcloud":
        input_topic = params.get("input_topic", "/jt128/front/points")
        output_topic = params.get("output_topic", "/jt128/front/points")
        restamp_on_receive = bool(params.get("restamp_on_receive", False))
        output_frame_id = str(params.get("output_frame_id", "") or "").strip()
        if input_topic != output_topic or restamp_on_receive or output_frame_id:
            return output_topic
        return input_topic
    return params.get("output_topic", "/jt128/front/points")


def _unitree_ddsc_env(runtime_mode):
    if runtime_mode != "real":
        return {}

    candidates = [
        "/opt/unitree_robotics/lib/x86_64/libddsc.so.0",
        "/unitree/opt/lib/libddsc.so.0",
    ]
    for candidate in candidates:
        if not os.path.exists(candidate):
            continue
        current = os.environ.get("LD_PRELOAD", "").strip()
        preload = candidate if not current else f"{candidate}:{current}"
        return {"LD_PRELOAD": preload}
    return {}


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(LaunchConfiguration("runtime_mode").perform(context))
    robot = LaunchConfiguration("robot").perform(context).strip() or "a2"
    robot_config_override = LaunchConfiguration("robot_config").perform(context)
    lidar = LaunchConfiguration("lidar").perform(context).strip()
    real_lidar_config_override = LaunchConfiguration("real_lidar_config").perform(context)
    camera = LaunchConfiguration("camera").perform(context).strip()
    real_camera_config_override = LaunchConfiguration("real_camera_config").perform(context)
    auto_start_explore = LaunchConfiguration("auto_start_explore").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    enable_nav2_bringup = LaunchConfiguration("enable_nav2_bringup").perform(context)
    enable_nav2_bringup_bool = as_bool(enable_nav2_bringup)
    enable_control_bridge = LaunchConfiguration("enable_control_bridge").perform(context)
    real_localization_mode = LaunchConfiguration("real_localization_mode").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = use_sim_time_for_mode(runtime_mode)
    use_sim_time_text = str(use_sim_time).lower()
    a2_system_share = get_package_share_directory("a2_system")
    bringup_share = get_package_share_directory("a2_bringup")
    robot_config_path = _resolve_config_path(
        a2_system_share=a2_system_share,
        override_path=robot_config_override,
        default_path="",
        candidate_paths=[f"{a2_system_share}/config/robots/{robot}.yaml"],
    )
    real_lidar_config_path = _resolve_config_path(
        a2_system_share=a2_system_share,
        override_path=real_lidar_config_override,
        default_path=f"{a2_system_share}/config/real_lidar.yaml",
        candidate_paths=[f"{a2_system_share}/config/lidars/{lidar}.yaml"] if lidar else [],
    )
    real_camera_config_path = _resolve_config_path(
        a2_system_share=a2_system_share,
        override_path=real_camera_config_override,
        default_path=f"{a2_system_share}/config/real_camera.yaml",
        candidate_paths=[f"{a2_system_share}/config/cameras/{camera}.yaml"] if camera else [],
    )
    unitree_ddsc_env = _unitree_ddsc_env(runtime_mode)
    real_lidar_topic = _real_lidar_consumer_topic(real_lidar_config_path, a2_system_share)
    slam_params = _load_yaml(f"{a2_system_share}/config/slam.yaml").get("slam_manager", {}).get(
        "ros__parameters", {}
    )
    map_representation = str(slam_params.get("primary_map_representation", "occupancy_grid_2d"))
    require_map_for_safety = map_representation != "pointcloud_map_3d"

    actions = [
        Node(
            package="a2_sdk_bridge",
            executable="a2_sdk_bridge_node",
            name="a2_sdk_bridge",
            additional_env=unitree_ddsc_env,
            parameters=[
                f"{a2_system_share}/config/a2_sdk.yaml",
                robot_config_path if robot_config_path else {},
                {
                "robot_profile": robot,
                "use_mock": False,
                "allow_loopback": False,
                "network_interface": network_interface,
                "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="a2_sdk_bridge",
            executable="a2_light_bridge_node",
            name="a2_light_bridge",
            additional_env=unitree_ddsc_env,
            parameters=[{
                "runtime_mode": runtime_mode,
                "use_mock": False,
                "allow_loopback": False,
                "network_interface": network_interface,
                "command_topic": "/a2/light/command",
                "lowcmd_topic": "rt/lowcmd",
                "send_repeat": 5,
                "send_hz": 10.0,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="a2_state_publisher",
            executable="a2_state_publisher_node",
            name="a2_state_publisher",
            parameters=[
                f"{a2_system_share}/config/state_bridge.yaml",
                robot_config_path if robot_config_path else {},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="a2_system",
            executable="task_manager.py",
            name="task_manager",
            parameters=[f"{a2_system_share}/config/task_manager.yaml", {
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="a2_control_bridge",
            executable="a2_control_bridge_node",
            name="a2_control_bridge",
            condition=IfCondition(enable_control_bridge),
            additional_env=unitree_ddsc_env,
            parameters=[
                f"{a2_system_share}/config/motion_limits.yaml",
                robot_config_path if robot_config_path else {},
                {
                "robot_profile": robot,
                "use_mock": False,
                "allow_loopback": False,
                "network_interface": network_interface,
                "runtime_mode": runtime_mode,
                "sim_cmd_topic": "",
                "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="safety_manager",
            executable="safety_supervisor",
            name="safety_supervisor",
            parameters=[f"{a2_system_share}/config/safety.yaml", {
                "lidar_topic": real_lidar_topic,
                "runtime_mode": runtime_mode,
                "latch_map_ready": enable_nav2_bringup_bool,
                "map_transient_local": enable_nav2_bringup_bool,
                "map_representation": map_representation,
                "require_map": require_map_for_safety,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="safety_manager",
            executable="real_readiness_monitor",
            name="real_readiness_monitor",
            parameters=[{
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/sensors.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time_text,
                "network_interface": network_interface,
                "robot_config": robot_config_path,
                "real_lidar_config": real_lidar_config_path,
                "real_camera_config": real_camera_config_path,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/slam.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "map": map_yaml,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/mapping.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "pointcloud_topic": real_lidar_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/localization.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "real_localization_mode": real_localization_mode,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/nav2.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "real_localization_mode": real_localization_mode,
                "map": map_yaml,
                "robot_config": robot_config_path,
                "real_lidar_config": real_lidar_config_path,
                "real_camera_config": real_camera_config_path,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/explore.launch.py"),
            launch_arguments={
                "auto_start": auto_start_explore,
                "use_sim_time": use_sim_time_text,
            }.items(),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("robot", default_value="a2"),
        DeclareLaunchArgument("robot_config", default_value=""),
        DeclareLaunchArgument("lidar", default_value=""),
        DeclareLaunchArgument("real_lidar_config", default_value=""),
        DeclareLaunchArgument("camera", default_value=""),
        DeclareLaunchArgument("real_camera_config", default_value=""),
        DeclareLaunchArgument("auto_start_explore", default_value="false"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("enable_control_bridge", default_value="false"),
        DeclareLaunchArgument("real_localization_mode", default_value="amcl"),
        DeclareLaunchArgument("map", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
