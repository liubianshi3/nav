# LEGACY 2D FILE — moved to launch/legacy/. See legacy/README.md.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import as_bool
import yaml


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _real_mapping_source(a2_system_share, runtime_mode):
    if runtime_mode != "real":
        return "front_lidar_pointcloud_3d"
    params = _load_yaml(f"{a2_system_share}/config/slam.yaml").get(
        "slam_manager", {}
    ).get("ros__parameters", {})
    return str(params.get("mapping_stack_profile", "slam_toolbox") or "slam_toolbox")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    runtime_mode_value = LaunchConfiguration("runtime_mode").perform(context)
    enable_nav2_bringup = as_bool(LaunchConfiguration("enable_nav2_bringup").perform(context))
    a2_system_share = get_package_share_directory("a2_system")
    mapping_source = _real_mapping_source(a2_system_share, runtime_mode_value)
    actions = []

    if not enable_nav2_bringup:
        if mapping_source == "front_lidar_pointcloud_3d":
            actions.append(
                Node(
                    package="map_manager",
                    executable="pointcloud_accumulator",
                    name="pointcloud_accumulator",
                    parameters=[f"{a2_system_share}/config/pointcloud_accumulator.yaml", {
                        "pointcloud_topic": pointcloud_topic,
                        "use_sim_time": use_sim_time,
                    }],
                )
            )
        elif mapping_source == "slam_toolbox":
            actions.extend([
                Node(
                    package="sensor_sync",
                    executable="pointcloud_to_laserscan",
                    name="pointcloud_to_laserscan",
                    parameters=[f"{a2_system_share}/config/pointcloud_to_scan.yaml", {
                        "input_topic": pointcloud_topic,
                        "use_sim_time": use_sim_time,
                    }],
                ),
                Node(
                    package="slam_toolbox",
                    executable="sync_slam_toolbox_node",
                    name="slam_toolbox",
                    parameters=[f"{a2_system_share}/config/slam_toolbox_mapping.yaml", {
                        "use_sim_time": use_sim_time,
                    }],
                ),
            ])
        elif mapping_source == "native_global_map":
            actions.append(
                Node(
                    package="map_manager",
                    executable="native_map_relay",
                    name="native_map_relay",
                    parameters=[f"{a2_system_share}/config/native_map_relay.yaml", {
                        "use_sim_time": use_sim_time,
                    }],
                )
            )
        else:
            actions.append(
                Node(
                    package="map_manager",
                    executable="occupancy_mapper",
                    name="occupancy_mapper",
                    parameters=[f"{a2_system_share}/config/occupancy_mapper.yaml", {
                        "pointcloud_topic": pointcloud_topic,
                        "runtime_mode": "real",
                        "use_sim_time": use_sim_time,
                    }],
                )
            )

    actions.append(
        Node(
            package="map_manager",
            executable="map_manager_node",
            name="map_manager",
            parameters=[f"{a2_system_share}/config/map_manager.yaml", {
                "runtime_mode": "real",
                "map_transient_local": enable_nav2_bringup,
                "use_sim_time": use_sim_time,
            }],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/a2/obstacle/points"),
        OpaqueFunction(function=_launch_setup),
    ])
