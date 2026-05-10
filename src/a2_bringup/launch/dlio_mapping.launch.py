from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    a2_system_share = get_package_share_directory("a2_system")
    start_dlio = _as_bool(LaunchConfiguration("start_dlio").perform(context))
    start_map_manager = _as_bool(LaunchConfiguration("start_map_manager").perform(context))
    start_watchdog = _as_bool(LaunchConfiguration("start_watchdog").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    pointcloud_topic = LaunchConfiguration("pointcloud_topic").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    dlio_config = LaunchConfiguration("dlio_config").perform(context)
    map_root = LaunchConfiguration("map_root").perform(context)

    actions = [
        Node(
            package="tf_manager",
            executable="static_tf_manager",
            name="jt128_static_tf_manager",
            parameters=[
                {
                    "extrinsics_file": f"{a2_system_share}/config/jt128_extrinsics.yaml",
                    "tf_file": f"{a2_system_share}/config/tf.yaml",
                    "base_height": 0.28,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    ]

    if start_dlio:
        try:
            get_package_share_directory("direct_lidar_inertial_odometry")
            actions.extend(
                [
                    LogInfo(
                        msg=(
                            "Starting DLIO for JT128: "
                            f"pointcloud={pointcloud_topic} imu={imu_topic} config={dlio_config}"
                        )
                    ),
                    Node(
                        package="direct_lidar_inertial_odometry",
                        executable="dlio_odom_node",
                        name="jt128_dlio_odom",
                        output="screen",
                        parameters=[dlio_config, {"use_sim_time": use_sim_time}],
                        remappings=[
                            ("pointcloud", pointcloud_topic),
                            ("imu", imu_topic),
                            ("odom", "/jt128/dlio/odom"),
                            ("pose", "/jt128/dlio/pose"),
                            ("path", "/jt128/dlio/path"),
                            ("kf_pose", "/jt128/dlio/keyframes"),
                            ("kf_cloud", "/jt128/dlio/pointcloud/keyframe"),
                            ("deskewed", "/jt128/dlio/pointcloud/deskewed"),
                        ],
                    ),
                    Node(
                        package="direct_lidar_inertial_odometry",
                        executable="dlio_map_node",
                        name="jt128_dlio_map",
                        output="screen",
                        parameters=[dlio_config, {"use_sim_time": use_sim_time}],
                        remappings=[
                            ("keyframes", "/jt128/dlio/pointcloud/keyframe"),
                            ("map", "/jt128/dlio/map_points"),
                            ("save_pcd", "/jt128/dlio/save_pcd"),
                        ],
                    ),
                ]
            )
        except PackageNotFoundError:
            actions.append(
                LogInfo(
                    msg=(
                        "direct_lidar_inertial_odometry is not installed. "
                        "Run `install_dlio_ros2.sh` or launch with start_dlio:=false "
                        "for driver-only validation."
                    )
                )
            )

    if start_watchdog and start_dlio:
        actions.append(
            Node(
                package="a2_system",
                executable="jt128_dlio_watchdog.py",
                name="jt128_dlio_watchdog",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/jt128/dlio/odom",
                        "max_position_norm": 50.0,
                        "max_abs_z": 5.0,
                        "max_linear_speed": 2.0,
                        "startup_grace_sec": 8.0,
                        "stop_on_fault": True,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )

    if start_map_manager:
        actions.append(
            Node(
                package="map_manager",
                executable="map_manager_node",
                name="map_manager",
                parameters=[
                    f"{a2_system_share}/config/map_manager.yaml",
                    {
                        "runtime_mode": "real",
                        "map_root": map_root,
                        "map_representation": "pointcloud_map_3d",
                        "pointcloud_topic_3d": "/jt128/dlio/map_points",
                        "pointcloud_fallback_topic_3d": pointcloud_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )

    return actions


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    bringup_share = get_package_share_directory("a2_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_dlio", default_value="true"),
            DeclareLaunchArgument("start_map_manager", default_value="true"),
            DeclareLaunchArgument("start_watchdog", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/jt128/front/points"),
            DeclareLaunchArgument("imu_topic", default_value="/jt128/front/imu"),
            DeclareLaunchArgument(
                "jt128_config",
                default_value=f"{a2_system_share}/config/jt128_front_hesai.yaml",
            ),
            DeclareLaunchArgument(
                "dlio_config",
                default_value=f"{a2_system_share}/config/dlio_jt128.yaml",
            ),
            DeclareLaunchArgument(
                "map_root", default_value=str(Path.home() / "a2_system_ws" / "runtime" / "maps")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{bringup_share}/launch/jt128_driver.launch.py"),
                condition=IfCondition(LaunchConfiguration("start_driver")),
                launch_arguments={
                    "config_path": LaunchConfiguration("jt128_config"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
