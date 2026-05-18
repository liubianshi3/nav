#!/usr/bin/env python3
"""Launch A2 JT128 perception simulation in Gazebo + RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("a2_bringup")
    urdf_path = os.path.join(bringup_share, "urdf", "a2_jt128_sim.urdf.xacro")
    world_path = os.path.join(bringup_share, "worlds", "a2_jt128_perception.world")
    rviz_path = os.path.join(bringup_share, "rviz", "a2_jt128_perception.rviz")

    gui = LaunchConfiguration("gui", default="true")
    use_rviz = LaunchConfiguration("rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    server_required = LaunchConfiguration("server_required", default="true")
    verbose = LaunchConfiguration("verbose", default="false")

    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    # ── Gazebo ──
    # server_required:=true makes the whole launch exit if gzserver fails,
    # preventing downstream nodes from appearing to run in an empty sim.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_path,
            "gui": gui,
            "server_required": server_required,
            "verbose": verbose,
        }.items(),
    )

    # ── Robot State Publisher ──
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ── Spawn robot (z=0 matches static TF map->base_link at origin) ──
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", "a2_jt128_sim",
                    "-topic", "robot_description",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                    "-timeout", "30.0",
                ],
                output="screen",
            ),
        ],
    )

    # ── Static map -> base_link TF (origin, matches spawn z=0) ──
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_base",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "map", "base_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Ground Segmentation C++ ──
    ground_seg = Node(
        package="a2_ground_segmentation_cpp",
        executable="ground_segmentation_cpp_node",
        name="ground_segmentation",
        output="screen",
        parameters=[
            {
                "input_topic": "/jt128/front/points",
                "ground_topic": "/a2/ground/points",
                "obstacle_topic": "/a2/obstacle/points",
                "traversability_topic": "/a2/traversability",
                "status_topic": "/a2/perception/ground_segmentation/status",
                "target_frame": "map",
                "input_min_range_m": 0.15,
                "self_filter_enabled": True,
                "self_filter_frame": "base_link",
                "self_filter_min_x": -0.45,
                "self_filter_max_x": 0.45,
                "self_filter_min_y": -0.35,
                "self_filter_max_y": 0.35,
                "self_filter_min_z": -0.20,
                "self_filter_max_z": 0.45,
                "traversability_width": 2000,
                "traversability_height": 2000,
                "traversability_origin_x": -100.0,
                "traversability_origin_y": -100.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ── Traversability -> Obstacle Cloud bridge ──
    trav_to_obs = Node(
        package="a2_system",
        executable="traversability_to_obstacle_cloud.py",
        name="traversability_to_obstacle_cloud",
        output="screen",
        parameters=[
            {
                "traversability_topic": "/a2/traversability",
                "output_topic": "/a2/traversability/obstacle_points",
                "output_frame": "base_link",
                "treat_unknown_as_obstacle": False,
                "local_window_enabled": True,
                "local_min_x": -1.0,
                "local_max_x": 6.0,
                "local_min_y": -4.0,
                "local_max_y": 4.0,
                "max_output_points": 20000,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ── RViz2 ──
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "server_required", default_value="true",
                description="Exit if gzserver fails to start"),
            DeclareLaunchArgument("verbose", default_value="false"),
            gazebo,
            robot_state_pub,
            static_tf,
            ground_seg,
            trav_to_obs,
            rviz,
            spawn_entity,
            LogInfo(msg="A2 JT128 perception simulation started."),
        ]
    )
