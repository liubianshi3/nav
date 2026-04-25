from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode, is_simulated_mode, as_bool


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(
        LaunchConfiguration("runtime_mode").perform(context),
        LaunchConfiguration("use_mock").perform(context),
    )
    enable_nav2_bringup = as_bool(LaunchConfiguration("enable_nav2_bringup").perform(context))
    real_localization_mode = (
        LaunchConfiguration("real_localization_mode").perform(context).strip() or "amcl"
    )
    map_yaml = LaunchConfiguration("map").perform(context).strip()
    gazebo_world = LaunchConfiguration("gazebo_world").perform(context).strip()
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_sim_time_text = str(use_sim_time).lower()
    a2_system_share = get_package_share_directory("a2_system")
    if runtime_mode == "gazebo" and not map_yaml:
        gazebo_bridge_share = get_package_share_directory("gazebo_bridge")
        if gazebo_world:
            world_name = Path(gazebo_world).stem
        else:
            world_name = "outdoor_research_park"
        if world_name == "office_house":
            map_yaml = f"{gazebo_bridge_share}/maps/office_house_map.yaml"
    else:
        if gazebo_world:
            world_name = Path(gazebo_world).stem
        else:
            world_name = ""
    use_manual_real_localization = (
        runtime_mode == "real"
        and enable_nav2_bringup
        and real_localization_mode == "manual_odom"
    )
    initial_pose_x = -7.2 if world_name == "outdoor_research_park" else 0.0
    initial_pose_y = 0.0
    initial_pose_yaw = 0.0
    actions = [
        Node(
            package="nav2_integration",
            executable="goal_bridge",
            name="goal_bridge",
            parameters=[f"{a2_system_share}/config/nav2.yaml", {
                "use_mock": runtime_mode != "real" and not enable_nav2_bringup,
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
    ]
    if is_simulated_mode(runtime_mode) and not enable_nav2_bringup:
        actions.append(
            Node(
                package="nav2_integration",
                executable="mock_nav_controller",
                name="mock_nav_controller",
                parameters=[f"{a2_system_share}/config/mock_nav_controller.yaml", {
                    "use_mock": runtime_mode == "mock",
                    "runtime_mode": runtime_mode,
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    if enable_nav2_bringup:
        actions.append(
            Node(
                package="mid360_wrapper",
                executable="pointcloud_to_laserscan",
                name="pointcloud_to_laserscan",
                parameters=[f"{a2_system_share}/config/pointcloud_to_scan.yaml", {
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    if runtime_mode == "gazebo" and enable_nav2_bringup:
        actions.append(
            Node(
                package="gazebo_bridge",
                executable="initial_pose_publisher",
                name="gazebo_initial_pose_publisher",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "x": initial_pose_x,
                    "y": initial_pose_y,
                    "yaw": initial_pose_yaw,
                    "use_zero_stamp": False,
                    "use_latest_odom_stamp": True,
                }],
            )
        )

    try:
        nav2_share = get_package_share_directory("nav2_bringup")
        if enable_nav2_bringup:
            if not map_yaml:
                actions.append(
                    LogInfo(
                        msg=(
                            "Nav2 bringup requested but no map yaml provided for the selected Gazebo world. "
                            "Run mapping first or pass map:=<saved_map.yaml>."
                        )
                    )
                )
                return actions
            if use_manual_real_localization:
                actions.extend([
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            f"{a2_system_share}/config/nav2_stack.yaml",
                            {
                                "use_sim_time": use_sim_time,
                                "yaml_filename": map_yaml,
                            },
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(f"{nav2_share}/launch/navigation_launch.py"),
                        launch_arguments={
                            "use_sim_time": use_sim_time_text,
                            "params_file": f"{a2_system_share}/config/nav2_stack.yaml",
                            "autostart": "true",
                            "use_composition": "False",
                            "use_respawn": "False",
                        }.items(),
                    ),
                ])
            else:
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(f"{nav2_share}/launch/bringup_launch.py"),
                        launch_arguments={
                            "use_sim_time": use_sim_time_text,
                            "params_file": f"{a2_system_share}/config/nav2_stack.yaml",
                            "autostart": "true",
                            "map": map_yaml,
                            "slam": "False",
                            "use_composition": "False",
                            "use_respawn": "False",
                        }.items(),
                    )
                )
    except PackageNotFoundError:
        pass

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_mock", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("real_localization_mode", default_value="amcl"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("gazebo_world", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
