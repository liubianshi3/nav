from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="odom"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("max_range", default_value="12.0"),
            DeclareLaunchArgument("sensor_max_range", default_value="12.0"),
            DeclareLaunchArgument("odom_topic", default_value="/jt128/dlio/odom"),
            DeclareLaunchArgument("cloud_topic", default_value="/jt128/front/points"),
            DeclareLaunchArgument("filtered_cloud_topic", default_value="/a2/octomap/cloud_in"),
            DeclareLaunchArgument("max_stamp_delta_sec", default_value="0.010"),
            DeclareLaunchArgument("save_path", default_value=""),
            DeclareLaunchArgument("save_period_sec", default_value="30.0"),
            DeclareLaunchArgument("pointcloud_min_z", default_value="-1.0"),
            DeclareLaunchArgument("pointcloud_max_z", default_value="2.5"),
            DeclareLaunchArgument("occupancy_min_z", default_value="0.12"),
            DeclareLaunchArgument("occupancy_max_z", default_value="1.2"),
            DeclareLaunchArgument("self_filter_enabled", default_value="true"),
            DeclareLaunchArgument("self_filter_min_x", default_value="-0.70"),
            DeclareLaunchArgument("self_filter_max_x", default_value="0.70"),
            DeclareLaunchArgument("self_filter_min_y", default_value="-0.45"),
            DeclareLaunchArgument("self_filter_max_y", default_value="0.45"),
            DeclareLaunchArgument("self_filter_min_z", default_value="-0.30"),
            DeclareLaunchArgument("self_filter_max_z", default_value="0.80"),
            DeclareLaunchArgument("min_range_m", default_value="0.20"),
            DeclareLaunchArgument(
                "lidar_to_base_translation",
                default_value="[0.33767, 0.0, 0.08134]",
            ),
            DeclareLaunchArgument(
                "lidar_to_base_rotation",
                default_value=(
                    "[0.0, 0.0, 1.0, "
                    "1.0, 0.0, 0.0, "
                    "0.0, 1.0, 0.0]"
                ),
            ),
            Node(
                package="a2_system",
                executable="octomap_mapping_node.py",
                name="octomap_mapping_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "filtered_cloud_topic": LaunchConfiguration("filtered_cloud_topic"),
                        "max_stamp_delta_sec": LaunchConfiguration("max_stamp_delta_sec"),
                        "save_path": LaunchConfiguration("save_path"),
                        "save_period_sec": LaunchConfiguration("save_period_sec"),
                        "self_filter_enabled": LaunchConfiguration("self_filter_enabled"),
                        "self_filter_min_x": LaunchConfiguration("self_filter_min_x"),
                        "self_filter_max_x": LaunchConfiguration("self_filter_max_x"),
                        "self_filter_min_y": LaunchConfiguration("self_filter_min_y"),
                        "self_filter_max_y": LaunchConfiguration("self_filter_max_y"),
                        "self_filter_min_z": LaunchConfiguration("self_filter_min_z"),
                        "self_filter_max_z": LaunchConfiguration("self_filter_max_z"),
                        "min_range_m": LaunchConfiguration("min_range_m"),
                        "max_range_m": LaunchConfiguration("sensor_max_range"),
                        "lidar_to_base_translation": LaunchConfiguration("lidar_to_base_translation"),
                        "lidar_to_base_rotation": LaunchConfiguration("lidar_to_base_rotation"),
                    }
                ],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "resolution": LaunchConfiguration("resolution"),
                        "sensor_model.max_range": LaunchConfiguration("max_range"),
                        "pointcloud_min_z": LaunchConfiguration("pointcloud_min_z"),
                        "pointcloud_max_z": LaunchConfiguration("pointcloud_max_z"),
                        "occupancy_min_z": LaunchConfiguration("occupancy_min_z"),
                        "occupancy_max_z": LaunchConfiguration("occupancy_max_z"),
                        "filter_ground_plane": False,
                        "compress_map": True,
                        "incremental_2D_projection": True,
                    }
                ],
                remappings=[
                    ("cloud_in", LaunchConfiguration("filtered_cloud_topic")),
                    ("octomap_binary", "/octomap_binary"),
                    ("octomap_full", "/octomap_full"),
                    ("projected_map", "/projected_map"),
                ],
            ),
        ]
    )
