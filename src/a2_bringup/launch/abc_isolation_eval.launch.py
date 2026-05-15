import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    a2_system_share = get_package_share_directory("a2_system")
    workspace = os.environ.get("A2_WORKSPACE", os.path.expanduser("~/a2_system_ws"))
    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
    if not rviz_config:
        rviz_config = os.path.join(a2_system_share, "config", "abc_isolation_eval.rviz")
    csv_path = LaunchConfiguration("csv_path").perform(context).strip()
    if not csv_path:
        csv_path = os.path.join(workspace, "runtime", "test_records", "abc_isolation_runs.csv")
    anchor_file = LaunchConfiguration("anchor_file").perform(context).strip()
    if not anchor_file:
        anchor_file = os.path.join(workspace, "runtime", "test_records", "abc_anchor_pose.json")

    eval_node = Node(
        package="a2_system",
        executable="abc_isolation_eval.py",
        name="abc_isolation_eval",
        output="screen",
        arguments=[
            "--mode",
            LaunchConfiguration("mode"),
            "--phase-label",
            LaunchConfiguration("phase_label"),
            "--map-id",
            LaunchConfiguration("map_id"),
            "--notes",
            LaunchConfiguration("notes"),
            "--pose-topic",
            LaunchConfiguration("pose_topic"),
            "--pose-msg-type",
            LaunchConfiguration("pose_msg_type"),
            "--goal-topic",
            LaunchConfiguration("goal_topic"),
            "--status-topic",
            LaunchConfiguration("status_topic"),
            "--initialpose-topic",
            LaunchConfiguration("initialpose_topic"),
            "--marker-topic",
            LaunchConfiguration("marker_topic"),
            "--path-topic",
            LaunchConfiguration("path_topic"),
            "--marker-frame",
            LaunchConfiguration("marker_frame"),
            "--csv-path",
            csv_path,
            "--anchor-file",
            anchor_file,
            "--position-tolerance-m",
            LaunchConfiguration("position_tolerance_m"),
            "--yaw-tolerance-deg",
            LaunchConfiguration("yaw_tolerance_deg"),
            "--goal-settle-sec",
            LaunchConfiguration("goal_settle_sec"),
            "--ready-stable-sec",
            LaunchConfiguration("ready_stable_sec"),
            "--max-pose-age-sec",
            LaunchConfiguration("max_pose_age_sec"),
            "--path-sample-distance-m",
            LaunchConfiguration("path_sample_distance_m"),
            "--path-sample-period-sec",
            LaunchConfiguration("path_sample_period_sec"),
            "--publish-period-sec",
            LaunchConfiguration("publish_period_sec"),
        ],
    )

    if (
        LaunchConfiguration("use_initialpose_as_anchor").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    ):
        eval_node.arguments.append("--use-initialpose-as-anchor")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="abc_isolation_eval_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    return [eval_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="navigation"),
            DeclareLaunchArgument("phase_label", default_value="A"),
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("notes", default_value=""),
            DeclareLaunchArgument("pose_topic", default_value="/a2/relocalization/pose"),
            DeclareLaunchArgument("pose_msg_type", default_value="pose_with_covariance"),
            DeclareLaunchArgument("goal_topic", default_value="/a2/nav3/goal_pose"),
            DeclareLaunchArgument("status_topic", default_value="/a2/nav2/status"),
            DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
            DeclareLaunchArgument("marker_topic", default_value="/a2/experiment/markers"),
            DeclareLaunchArgument("path_topic", default_value="/a2/experiment/path"),
            DeclareLaunchArgument("marker_frame", default_value="map"),
            DeclareLaunchArgument("csv_path", default_value=""),
            DeclareLaunchArgument("anchor_file", default_value=""),
            DeclareLaunchArgument("position_tolerance_m", default_value="0.30"),
            DeclareLaunchArgument("yaw_tolerance_deg", default_value="15.0"),
            DeclareLaunchArgument("goal_settle_sec", default_value="1.0"),
            DeclareLaunchArgument("ready_stable_sec", default_value="1.5"),
            DeclareLaunchArgument("max_pose_age_sec", default_value="2.0"),
            DeclareLaunchArgument("path_sample_distance_m", default_value="0.05"),
            DeclareLaunchArgument("path_sample_period_sec", default_value="0.50"),
            DeclareLaunchArgument("publish_period_sec", default_value="0.20"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            DeclareLaunchArgument("use_initialpose_as_anchor", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
