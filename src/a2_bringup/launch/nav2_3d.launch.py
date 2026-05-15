import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_a2_system_script(script_name: str) -> str:
    package_prefix = Path(get_package_prefix("a2_system"))
    install_path = package_prefix / "lib" / "a2_system" / script_name
    if install_path.is_file() and os.access(install_path, os.X_OK):
        return str(install_path)

    workspace = package_prefix.parent.parent
    source_path = workspace / "src" / "a2_system" / "scripts" / script_name
    if source_path.is_file() and os.access(source_path, os.X_OK):
        return str(source_path)

    raise FileNotFoundError(
        f"unable to resolve executable for a2_system script '{script_name}' "
        f"(checked {install_path} and {source_path})"
    )


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    traversability_bridge = _resolve_a2_system_script("traversability_to_obstacle_cloud.py")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")

    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        nav2_share = _gpsd("nav2_bringup")
    except Exception:
        return LaunchDescription([
            LogInfo(msg="nav2_bringup not found; cannot launch nav2_3d stack."),
        ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value="",
                              description="Path to 2D projected map YAML file"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "enable_global_ekf_debug",
            default_value="false",
            description="Run the map-frame EKF debug topic alongside the local odom chain",
        ),

        LogInfo(
            msg=(
                "Starting Nav2 3D navigation stack. "
                "Requires: NDT localization, /odometry/local, ground_segmentation. "
                "map argument must point to a 2D projection of the PCD map."
            )
        ),

        # Optional map-frame EKF debug topic. The local odom mainline is started
        # by jt128_3d_navigation.launch.py and remains the primary Nav2 odom feed.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("a2_bringup"),
                    "launch",
                    "ekf.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "enable_ekf": LaunchConfiguration("enable_global_ekf_debug"),
                "output_topic": "/odometry/global_debug",
                "enable_odom_to_pose_bridge": "false",
            }.items(),
        ),

        # Traversability grid → obstacle pointcloud bridge
        Node(
            executable=traversability_bridge,
            name="traversability_to_obstacle_cloud",
            parameters=[{
                "traversability_obstacle_threshold": 90,
                "publish_hz": 2.0,
                "obstacle_z": 0.15,
                "traversability_topic": "/a2/traversability",
                "output_topic": "/a2/traversability/obstacle_points",
                "output_frame": "map",
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),

        # Goal bridge — converts exploration goals to Nav2 NavigateToPose
        Node(
            package="nav2_integration",
            executable="goal_bridge",
            name="goal_bridge",
            parameters=[f"{a2_system_share}/config/nav2.yaml", {
                "runtime_mode": "real",
                "navigation_backend": "nav2",
                "pose_goal_topic": "/a2/nav3/goal_pose",
                "map_frame": "map",
                "use_sim_time": use_sim_time,
            }],
        ),

        # Copy custom BT XML to well-known absolute path so Nav2 bt_navigator can find it.
        # (bt_xml_filename in nav2_3d.yaml points to /tmp/a2_navigate_3d.xml)
        ExecuteProcess(
            cmd=['cp', '-f',
                 os.path.join(a2_system_share, 'config', 'a2_navigate_3d.xml'),
                 '/tmp/a2_navigate_3d.xml'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['cp', '-f',
                 os.path.join(a2_system_share, 'config', 'a2_navigate_through_poses_3d.xml'),
                 '/tmp/a2_navigate_through_poses_3d.xml'],
            output='screen',
        ),

        # Map server only. NDT owns map->odom localization, so we avoid AMCL.
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                f"{a2_system_share}/config/nav2_3d.yaml",
                {
                    "yaml_filename": map_yaml,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[
                f"{a2_system_share}/config/nav2_3d.yaml",
                {
                    "autostart": True,
                    "node_names": ["map_server"],
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),

        # Nav2 planning/control stack. Localization is provided by NDT + EKF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{nav2_share}/launch/navigation_launch.py"),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": f"{a2_system_share}/config/nav2_3d.yaml",
                "autostart": "true",
                "use_composition": "False",
                "use_respawn": "False",
            }.items(),
        ),
    ])
