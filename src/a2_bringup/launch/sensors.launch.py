import json
import os
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from a2_bringup.runtime_mode import normalize_runtime_mode


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _run(command):
    return subprocess.run(command, capture_output=True, text=True, check=False)


def _interface_ipv4(interface_name):
    if not interface_name:
        return ""
    result = _run(["ip", "-4", "-o", "addr", "show", "dev", interface_name, "scope", "global"])
    if result.returncode != 0:
        return ""
    for line in result.stdout.splitlines():
        fields = line.split()
        if "inet" not in fields:
            continue
        address = fields[fields.index("inet") + 1]
        return address.split("/")[0]
    return ""


def _fallback_sensor_ip(network_cfg):
    configured = network_cfg.get("network", {}).get("mid360_host_ip", "")
    if configured:
        return configured
    sensor_subnet = network_cfg.get("network", {}).get("a2_sensor_subnet", "192.168.124.0/24")
    prefix = sensor_subnet.split("/")[0].rsplit(".", 1)[0]
    return f"{prefix}.10"


def _render_mid360_runtime_config(a2_system_share, runtime_dir, interface_name):
    network_cfg = _load_yaml(os.path.join(a2_system_share, "config", "network.yaml"))
    livox_cfg = _load_yaml(os.path.join(a2_system_share, "config", "livox_mid360_driver.yaml"))

    network = network_cfg.get("network", {})
    host_ip = _interface_ipv4(interface_name) or _fallback_sensor_ip(network_cfg)
    lidar_ip = network.get("mid360_ip", "192.168.124.20")

    runtime_dir.mkdir(parents=True, exist_ok=True)
    json_path = runtime_dir / "MID360_config.json"

    rendered_json = {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": int(network.get("mid360_command_port", 56100)),
                "push_msg_port": int(network.get("mid360_push_port", 56200)),
                "point_data_port": int(network.get("mid360_data_port", 56300)),
                "imu_data_port": int(network.get("mid360_imu_port", 56400)),
                "log_data_port": int(network.get("mid360_log_port", 56500)),
            },
            "host_net_info": {
                "cmd_data_ip": host_ip,
                "cmd_data_port": int(network.get("mid360_host_command_port", 56101)),
                "push_msg_ip": host_ip,
                "push_msg_port": int(network.get("mid360_host_push_port", 56201)),
                "point_data_ip": host_ip,
                "point_data_port": int(network.get("mid360_host_data_port", 56301)),
                "imu_data_ip": host_ip,
                "imu_data_port": int(network.get("mid360_host_imu_port", 56401)),
                "log_data_ip": "",
                "log_data_port": int(network.get("mid360_host_log_port", 56501)),
            },
        },
        "lidar_configs": [
            {
                "ip": lidar_ip,
                "pcl_data_type": 1,
                "pattern_mode": 0,
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }
    json_path.write_text(json.dumps(rendered_json, indent=2), encoding="utf-8")

    driver_cfg = livox_cfg.get("mid360_driver", {})
    driver_params = dict(driver_cfg.get("driver", {}))
    relay_cfg = dict(driver_cfg.get("relay", {}))
    driver_params["user_config_path"] = str(json_path)
    driver_params.setdefault("frame_id", relay_cfg.get("frame_id", "lidar_link"))
    driver_params.setdefault("lvx_file_path", "")
    driver_params.setdefault("cmdline_input_bd_code", "")

    return {
        "json_path": str(json_path),
        "host_ip": host_ip,
        "lidar_ip": lidar_ip,
        "driver_params": driver_params,
        "relay_cfg": relay_cfg,
    }


def _load_real_lidar_config(a2_system_share):
    cfg = _load_yaml(os.path.join(a2_system_share, "config", "real_lidar.yaml"))
    return cfg.get("real_lidar", {}).get("ros__parameters", {})


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    runtime_mode = normalize_runtime_mode(
        LaunchConfiguration("runtime_mode").perform(context),
        LaunchConfiguration("use_mock").perform(context),
    )
    use_mock = runtime_mode == "mock"
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    network_interface = LaunchConfiguration("network_interface").perform(context).strip()
    gazebo_world = LaunchConfiguration("gazebo_world").perform(context).strip()
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context).strip()
    gazebo_paused = LaunchConfiguration("gazebo_paused").perform(context).strip()
    a2_system_share = get_package_share_directory("a2_system")
    bringup_share = get_package_share_directory("a2_bringup")
    runtime_dir = Path.home() / "a2_system_ws" / "runtime" / "generated"
    diagnostic_only = os.environ.get("A2_REAL_DIAGNOSTIC_ONLY", "0") == "1"
    real_lidar_cfg = _load_real_lidar_config(a2_system_share)
    real_lidar_profile = real_lidar_cfg.get("profile", "livox_mid360")
    real_lidar_driver_mode = real_lidar_cfg.get("driver_mode", "")
    real_lidar_input_topic = real_lidar_cfg.get("input_topic", "/unitree/slam_lidar/points1")
    real_lidar_output_topic = real_lidar_cfg.get("output_topic", "/mid360/points")
    direct_pointcloud_mode = (
        real_lidar_driver_mode == "external_pointcloud"
        or real_lidar_profile == "unitree_native_fused"
    )
    guard_pointcloud_topic = (
        real_lidar_input_topic if direct_pointcloud_mode else real_lidar_output_topic
    )
    guard_stale_timeout = float(real_lidar_cfg.get("stale_timeout_sec", 1.0))
    livox_available = True
    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError:
        livox_available = False
    guard_driver_available = (
        livox_available
        or real_lidar_driver_mode == "external_pointcloud"
        or real_lidar_profile == "unitree_native_fused"
    )

    actions = [
        Node(
            package="tf_manager",
            executable="static_tf_manager",
            name="static_tf_manager",
            parameters=[{
                "extrinsics_file": f"{a2_system_share}/config/extrinsics.yaml",
                "tf_file": f"{a2_system_share}/config/tf.yaml",
                "base_height": 0.28,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="sensor_sync",
            executable="sync_monitor",
            name="sync_monitor",
            parameters=[f"{a2_system_share}/config/sensor_sync.yaml", {
                "pointcloud_topic": guard_pointcloud_topic,
                "use_mock": use_mock,
                "runtime_mode": runtime_mode,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="mid360_wrapper",
            executable="mid360_driver_guard",
            name="mid360_driver_guard",
            parameters=[{
                "use_mock": use_mock,
                "runtime_mode": runtime_mode,
                "driver_available": guard_driver_available,
                "pointcloud_topic": guard_pointcloud_topic,
                "stale_timeout_sec": guard_stale_timeout,
                "connected_topic": "/a2/lidar/connected",
                "status_topic": "/a2/lidar/status",
                "status_label": "lidar",
                "use_sim_time": use_sim_time,
            }],
        ),
    ]

    if use_mock:
        actions.append(
            Node(
                package="mid360_wrapper",
                executable="mock_mid360_publisher",
                name="mock_mid360_publisher",
                parameters=[f"{a2_system_share}/config/mid360.yaml", {"use_sim_time": use_sim_time}],
            )
        )
        return actions

    if runtime_mode == "gazebo":
        gazebo_launch_args = {
            "gui": gazebo_gui,
            "paused": gazebo_paused,
            "use_sim_time": str(use_sim_time).lower(),
        }
        if gazebo_world:
            gazebo_launch_args["world"] = gazebo_world
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{bringup_share}/launch/gazebo_sim.launch.py"),
                launch_arguments=gazebo_launch_args.items(),
            )
        )
        return actions

    if direct_pointcloud_mode:
        input_topic = real_lidar_input_topic
        output_topic = real_lidar_output_topic
        output_frame_id = real_lidar_cfg.get("output_frame_id", "lidar_link")
        restamp_on_receive = bool(real_lidar_cfg.get("restamp_on_receive", False))
        actions.append(
            LogInfo(
                msg=(
                    f"Using robot-native fused lidar input_topic={input_topic} "
                    f"direct_consumer_topic={guard_pointcloud_topic} compatibility_output={output_topic} "
                    f"frame_id={output_frame_id} restamp_on_receive={restamp_on_receive}"
                )
            )
        )
        actions.append(
            Node(
                package="mid360_wrapper",
                executable="pointcloud_frame_relay",
                name="pointcloud_frame_relay",
                parameters=[{
                    "input_topic": input_topic,
                    "output_topic": output_topic,
                    "frame_id": output_frame_id,
                    "restamp_on_receive": restamp_on_receive,
                }],
            )
        )
        return actions

    if not livox_available:
        actions.append(LogInfo(msg="livox_ros_driver2 not found. Real MID360 launch will stay offline."))
        return actions

    if diagnostic_only:
        actions.append(LogInfo(msg="Real stack is in diagnostic mode. Livox MID360 driver launch is deferred until the wired interface is ready."))
        return actions

    rendered = _render_mid360_runtime_config(a2_system_share, runtime_dir, network_interface)
    driver_params = rendered["driver_params"]
    relay_cfg = rendered["relay_cfg"]
    prefer_custom_msg = _as_bool(relay_cfg.get("prefer_custom_msg", True))
    pointcloud_output_topic = relay_cfg.get("pointcloud_output_topic", "/mid360/points")
    relay_frame = relay_cfg.get("frame_id", "lidar_link")

    actions.append(
        LogInfo(
            msg=(
                f"MID360 real config host_ip={rendered['host_ip']} lidar_ip={rendered['lidar_ip']} "
                f"user_config_path={rendered['json_path']}"
            )
        )
    )
    actions.append(
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="mid360_driver",
            output="screen",
            additional_env={
                "LD_LIBRARY_PATH": f"{os.environ.get('LD_LIBRARY_PATH', '')}:/usr/local/lib"
            },
            parameters=[driver_params],
        )
    )

    if int(driver_params.get("xfer_format", 1)) == 1 and prefer_custom_msg:
        actions.append(
            Node(
                package="mid360_wrapper",
                executable="livox_custom_to_pointcloud",
                name="livox_custom_to_pointcloud",
                parameters=[{
                    "input_topic": relay_cfg.get("custom_msg_topic", "/livox/lidar"),
                    "output_topic": pointcloud_output_topic,
                    "frame_id": relay_frame,
                }],
            )
        )
    else:
        actions.append(
            Node(
                package="mid360_wrapper",
                executable="pointcloud_frame_relay",
                name="pointcloud_frame_relay",
                parameters=[{
                    "input_topic": relay_cfg.get("pointcloud_input_topic", "/livox/lidar"),
                    "output_topic": pointcloud_output_topic,
                    "frame_id": relay_frame,
                    "restamp_on_receive": bool(relay_cfg.get("restamp_on_receive", False)),
                }],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime_mode", default_value=""),
        DeclareLaunchArgument("use_mock", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("gazebo_world", default_value=""),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("gazebo_paused", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
