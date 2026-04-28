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


def _real_lidar_consumer_topic(a2_system_share, runtime_mode):
    if runtime_mode != "real":
        return "/mid360/points"
    params = _load_yaml(f"{a2_system_share}/config/real_lidar.yaml").get("real_lidar", {}).get(
        "ros__parameters", {}
    )
    profile = params.get("profile", "")
    driver_mode = params.get("driver_mode", "")
    if profile == "unitree_native_fused" or driver_mode == "external_pointcloud":
        return params.get("input_topic", "/unitree/slam_lidar/points1")
    return params.get("output_topic", "/mid360/points")


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
    use_mock_value = LaunchConfiguration("use_mock").perform(context)
    runtime_mode = normalize_runtime_mode(
        LaunchConfiguration("runtime_mode").perform(context), use_mock_value
    )
    auto_start_explore = LaunchConfiguration("auto_start_explore").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    enable_nav2_bringup = LaunchConfiguration("enable_nav2_bringup").perform(context)
    enable_nav2_bringup_bool = as_bool(enable_nav2_bringup)
    enable_control_bridge = LaunchConfiguration("enable_control_bridge").perform(context)
    real_localization_mode = LaunchConfiguration("real_localization_mode").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    gazebo_world = LaunchConfiguration("gazebo_world").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    gazebo_paused = LaunchConfiguration("gazebo_paused").perform(context)
    use_sim_time = use_sim_time_for_mode(runtime_mode)
    use_sim_time_text = str(use_sim_time).lower()
    use_mock = runtime_mode == "mock"
    use_mock_text = str(use_mock).lower()
    a2_system_share = get_package_share_directory("a2_system")
    bringup_share = get_package_share_directory("a2_bringup")
    unitree_ddsc_env = _unitree_ddsc_env(runtime_mode)
    real_lidar_topic = _real_lidar_consumer_topic(a2_system_share, runtime_mode)

    actions = []
    if runtime_mode == "gazebo":
        actions.append(
            Node(
                package="gazebo_bridge",
                executable="gazebo_state_adapter",
                name="gazebo_state_adapter",
                parameters=[{
                    "runtime_mode": runtime_mode,
                    "use_sim_time": True,
                    "odom_topic": "/gazebo/odom",
                    "imu_topic": "/gazebo/imu",
                    "state_topic": "/a2/raw_state",
                    "sdk_connected_topic": "/a2/sdk/connected",
                    "sdk_status_topic": "/a2/sdk/status",
                }],
            )
        )
    else:
        actions.append(
            Node(
                package="a2_sdk_bridge",
                executable="a2_sdk_bridge_node",
                name="a2_sdk_bridge",
                additional_env=unitree_ddsc_env,
                parameters=[f"{a2_system_share}/config/a2_sdk.yaml", {
                    "use_mock": use_mock,
                    "allow_loopback": use_mock,
                    "network_interface": network_interface,
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    actions.extend([
        Node(
            package="a2_state_publisher",
            executable="a2_state_publisher_node",
            name="a2_state_publisher",
            parameters=[f"{a2_system_share}/config/state_bridge.yaml", {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="a2_system",
            executable="task_manager.py",
            name="task_manager",
            parameters=[f"{a2_system_share}/config/task_manager.yaml", {
                "use_mock": use_mock,
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
            parameters=[f"{a2_system_share}/config/motion_limits.yaml", {
                "use_mock": use_mock,
                "allow_loopback": use_mock,
                "network_interface": network_interface,
                "runtime_mode": runtime_mode,
                "sim_cmd_topic": "/gazebo/cmd_vel" if runtime_mode == "gazebo" else "",
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="safety_manager",
            executable="safety_supervisor",
            name="safety_supervisor",
            parameters=[f"{a2_system_share}/config/safety.yaml", {
                "lidar_topic": real_lidar_topic,
                "use_mock": use_mock,
                "runtime_mode": runtime_mode,
                "latch_map_ready": enable_nav2_bringup_bool,
                "map_transient_local": enable_nav2_bringup_bool,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="safety_manager",
            executable="real_readiness_monitor",
            name="real_readiness_monitor",
            parameters=[{
                "use_mock": use_mock,
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/sensors.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_mock": use_mock_text,
                "use_sim_time": use_sim_time_text,
                "network_interface": network_interface,
                "gazebo_world": gazebo_world,
                "gazebo_gui": gazebo_gui,
                "gazebo_paused": gazebo_paused,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/slam.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_mock": use_mock_text,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "map": map_yaml,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/mapping.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_mock": use_mock_text,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "pointcloud_topic": real_lidar_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/localization.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_mock": use_mock_text,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "real_localization_mode": real_localization_mode,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/nav2.launch.py"),
            launch_arguments={
                "runtime_mode": runtime_mode,
                "use_mock": use_mock_text,
                "use_sim_time": use_sim_time_text,
                "enable_nav2_bringup": enable_nav2_bringup,
                "real_localization_mode": real_localization_mode,
                "map": map_yaml,
                "gazebo_world": gazebo_world,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{bringup_share}/launch/explore.launch.py"),
            launch_arguments={
                "auto_start": auto_start_explore,
                "use_sim_time": use_sim_time_text,
            }.items(),
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_mock", default_value="true"),
        DeclareLaunchArgument("auto_start_explore", default_value="false"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("enable_control_bridge", default_value="false"),
        DeclareLaunchArgument("real_localization_mode", default_value="amcl"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("gazebo_world", default_value=""),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("gazebo_paused", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
