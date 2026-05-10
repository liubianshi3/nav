import os
import tempfile
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode, as_bool


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _real_lidar_params(real_lidar_config_path):
    return _load_yaml(real_lidar_config_path).get("real_lidar", {}).get("ros__parameters", {})


def _real_camera_params(real_camera_config_path):
    return _load_yaml(real_camera_config_path).get("real_camera", {}).get("ros__parameters", {})


def _real_lidar_consumer_topic(params):
    profile = params.get("profile", "")
    driver_mode = params.get("driver_mode", "")
    input_topic = params.get("input_topic", "/jt128/front/points")
    output_topic = params.get("output_topic", "/jt128/front/points")
    if profile == "unitree_native_fused" or driver_mode == "external_pointcloud":
        restamp_on_receive = bool(params.get("restamp_on_receive", False))
        output_frame_id = str(params.get("output_frame_id", "") or "").strip()
        if input_topic != output_topic or restamp_on_receive or output_frame_id:
            return output_topic
        return input_topic
    return output_topic


def _pointcloud_scan_input_topic(real_lidar_config_path):
    params = _real_lidar_params(real_lidar_config_path)
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


def _real_camera_consumer_topic(params):
    enabled = bool(params.get("enabled", False))
    if not enabled:
        return ""
    input_topic = str(params.get("input_pointcloud_topic", "") or "").strip()
    output_topic = str(params.get("output_pointcloud_topic", "") or "").strip()
    output_frame_id = str(params.get("output_frame_id", "") or "").strip()
    restamp_on_receive = bool(params.get("restamp_on_receive", False))
    if not input_topic:
        return ""
    if output_topic and (input_topic != output_topic or restamp_on_receive or output_frame_id):
        return output_topic
    return input_topic


def _append_observation_source(existing, source_name):
    tokens = [token for token in str(existing or "").replace(",", " ").split(" ") if token]
    if source_name in tokens:
        return " ".join(tokens)
    tokens.append(source_name)
    return " ".join(tokens)


def _patch_voxel_layer(cfg, scope, lidar_topic, camera_topic, camera_params):
    try:
        layer = cfg[scope][scope]["ros__parameters"]["voxel_layer"]
    except (TypeError, KeyError):
        return

    try:
        layer["jt128_points"]["topic"] = lidar_topic
    except (TypeError, KeyError):
        pass

    if not camera_topic:
        return
    if not bool(camera_params.get("use_for_costmap", False)):
        return

    layer["observation_sources"] = _append_observation_source(layer.get("observation_sources", ""), "depth_points")
    depth_points = layer.get("depth_points", {})
    depth_points["topic"] = camera_topic
    depth_points["data_type"] = "PointCloud2"
    depth_points["marking"] = True
    depth_points["clearing"] = True
    depth_points["min_obstacle_height"] = float(camera_params.get("min_obstacle_height", 0.05))
    depth_points["max_obstacle_height"] = float(camera_params.get("max_obstacle_height", 2.0))
    depth_points["obstacle_min_range"] = float(camera_params.get("obstacle_min_range", 0.2))
    depth_points["obstacle_max_range"] = float(camera_params.get("obstacle_max_range", 4.0))
    depth_points["raytrace_min_range"] = float(camera_params.get("raytrace_min_range", 0.2))
    depth_points["raytrace_max_range"] = float(camera_params.get("raytrace_max_range", 5.0))
    layer["depth_points"] = depth_points


def _patch_costmap_voxel_topics(cfg, lidar_topic, camera_topic, camera_params):
    _patch_voxel_layer(cfg, "global_costmap", lidar_topic, camera_topic, camera_params)
    _patch_voxel_layer(cfg, "local_costmap", lidar_topic, camera_topic, camera_params)
    return cfg


def _write_nav2_params(original_params_path, lidar_topic, camera_topic, camera_params):
    cfg = _load_yaml(original_params_path)
    cfg = _patch_costmap_voxel_topics(cfg, lidar_topic, camera_topic, camera_params)
    handle = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", prefix="nav2_stack_", suffix=".yaml", delete=False
    )
    with handle:
        yaml.safe_dump(cfg, handle, sort_keys=False)
    return handle.name


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(LaunchConfiguration("runtime_mode").perform(context))
    enable_nav2_bringup = as_bool(LaunchConfiguration("enable_nav2_bringup").perform(context))
    real_localization_mode = (
        LaunchConfiguration("real_localization_mode").perform(context).strip() or "amcl"
    )
    map_yaml = LaunchConfiguration("map").perform(context).strip()
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_sim_time_text = str(use_sim_time).lower()
    real_lidar_config_path = LaunchConfiguration("real_lidar_config").perform(context).strip()
    real_camera_config_path = LaunchConfiguration("real_camera_config").perform(context).strip()
    a2_system_share = get_package_share_directory("a2_system")
    if not real_lidar_config_path:
        real_lidar_config_path = os.path.join(a2_system_share, "config", "real_lidar.yaml")
    if not real_camera_config_path:
        real_camera_config_path = os.path.join(a2_system_share, "config", "real_camera.yaml")
    real_lidar = _real_lidar_params(real_lidar_config_path)
    real_camera = _real_camera_params(real_camera_config_path)
    pointcloud_scan_input_topic = _pointcloud_scan_input_topic(real_lidar_config_path)
    lidar_costmap_topic = _real_lidar_consumer_topic(real_lidar)
    camera_costmap_topic = _real_camera_consumer_topic(real_camera)
    nav2_params_source = f"{a2_system_share}/config/nav2_stack.yaml"
    nav2_params_file = _write_nav2_params(nav2_params_source, lidar_costmap_topic, camera_costmap_topic, real_camera)
    slam_params = _load_yaml(f"{a2_system_share}/config/slam.yaml").get("slam_manager", {}).get(
        "ros__parameters", {}
    )
    navigation_representation = str(slam_params.get("navigation_representation", "occupancy_grid_2d"))
    use_3d_navigation = runtime_mode == "real" and navigation_representation == "pointcloud_map_3d"
    use_manual_real_localization = (
        runtime_mode == "real"
        and enable_nav2_bringup
        and real_localization_mode == "manual_odom"
    )
    actions = [
        Node(
            package="nav2_integration",
            executable="goal_bridge",
            name="goal_bridge",
            parameters=[f"{a2_system_share}/config/nav2.yaml", {
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
    ]

    if enable_nav2_bringup and not use_3d_navigation:
        actions.append(
            Node(
                package="sensor_sync",
                executable="pointcloud_to_laserscan",
                name="pointcloud_to_laserscan",
                parameters=[f"{a2_system_share}/config/pointcloud_to_scan.yaml", {
                    "input_topic": pointcloud_scan_input_topic,
                    "use_sim_time": use_sim_time,
                }],
            )
        )

    try:
        nav2_share = get_package_share_directory("nav2_bringup")
        if enable_nav2_bringup and not use_3d_navigation:
            if not map_yaml:
                actions.append(
                    LogInfo(
                        msg=(
                            "Nav2 bringup requested but no map yaml provided for the current stack. "
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
                            nav2_params_file,
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
                            "params_file": nav2_params_file,
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
                            "params_file": nav2_params_file,
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
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_nav2_bringup", default_value="false"),
        DeclareLaunchArgument("real_localization_mode", default_value="amcl"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("robot_config", default_value=""),
        DeclareLaunchArgument("real_lidar_config", default_value=""),
        DeclareLaunchArgument("real_camera_config", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
