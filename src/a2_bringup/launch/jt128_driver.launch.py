import os
import shutil
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    config_path = LaunchConfiguration("config_path").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    node_name = LaunchConfiguration("node_name").perform(context).strip() or "jt128_hesai_driver"
    workspace = Path(os.environ.get("A2_WORKSPACE", str(Path.home() / "a2_system_ws")))
    driver_cwd = workspace / "runtime" / "jt128_hesai_driver"
    driver_config = driver_cwd / "config_files" / "hs_lidar_jt128" / "config.yaml"

    try:
        get_package_share_directory("hesai_ros_driver")
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "hesai_ros_driver package is not visible. Source the Hesai/Unitree "
                    "driver workspace before launching JT128, for example "
                    "`source /home/unitree/graph_pid_ws/install/setup.bash`."
                )
            )
        ]

    driver_config.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(config_path, driver_config)

    return [
        LogInfo(
            msg=(
                "Starting JT128 Hesai driver "
                f"cwd={driver_cwd} config={driver_config} source_config={config_path}"
            )
        ),
        Node(
            package="hesai_ros_driver",
            executable="hesai_ros_driver_node",
            name=node_name,
            output="screen",
            cwd=str(driver_cwd),
            parameters=[{"config_path": config_path, "use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description():
    a2_system_share = get_package_share_directory("a2_system")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=f"{a2_system_share}/config/jt128_front_hesai.yaml",
            ),
            DeclareLaunchArgument("node_name", default_value="jt128_hesai_driver"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
