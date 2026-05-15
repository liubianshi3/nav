from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    a2_system_share = get_package_share_directory('a2_system')
    ndt_share = get_package_share_directory('autoware_ndt_scan_matcher')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    base_ndt_param_file = LaunchConfiguration('base_ndt_param_file')
    ndt_param_file = LaunchConfiguration('ndt_param_file')
    auto_activate_ndt = LaunchConfiguration('auto_activate_ndt', default='true')
    odom_topic = LaunchConfiguration('odom_topic')
    align_initial_pose_stamp_to_cloud = LaunchConfiguration('align_initial_pose_stamp_to_cloud')
    max_map_to_odom_translation_step = LaunchConfiguration('max_map_to_odom_translation_step')
    max_map_to_odom_rotation_step_deg = LaunchConfiguration('max_map_to_odom_rotation_step_deg')

    # NDT Scan Matcher Node
    ndt_node = Node(
        package='autoware_ndt_scan_matcher',
        executable='autoware_ndt_scan_matcher_node',
        name='ndt_scan_matcher',
        parameters=[
            base_ndt_param_file,
            ndt_param_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('points_raw', '/jt128/front/points'),
            ('pointcloud_map', '/a2/map/pointcloud_3d'),
            ('ekf_pose_with_covariance', '/a2/ndt/open_loop_pose'),
        ],
        output='screen'
    )

    # A2 NDT Adapter Node
    adapter_node = Node(
        package='a2_ndt_adapter',
        executable='ndt_adapter_node',
        name='ndt_adapter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'live_cloud_topic': '/jt128/front/points',
            'odom_topic': odom_topic,
            'map_topic': '/a2/map/pointcloud_3d',
            'pose_topic': '/a2/relocalization/pose',
            'status_topic': '/a2/relocalization/status',
            'score_topic': 'transform_probability',
            'score_threshold': 2.3,
            'score_min_is_good': True,
            'score_timeout_sec': 12.0,
            'map_service_max_radius': 25.0,
            'map_service_margin_m': 3.0,
            'map_service_max_points': 60000,
            'ndt_initial_pose_topic': '/a2/ndt/open_loop_pose',
            'align_initial_pose_stamp_to_cloud': ParameterValue(align_initial_pose_stamp_to_cloud, value_type=bool),
            'max_map_to_odom_translation_step': ParameterValue(max_map_to_odom_translation_step, value_type=float),
            'max_map_to_odom_rotation_step_deg': ParameterValue(max_map_to_odom_rotation_step_deg, value_type=float),
        }],
        output='screen'
    )

    # Auto Activation
    trigger_cmd = [
        'ros2', 'service', 'call', '/trigger_node_srv',
        'std_srvs/srv/SetBool', '"{data: true}"'
    ]
    auto_activate = ExecuteProcess(
        cmd=trigger_cmd,
        condition=IfCondition(auto_activate_ndt),
        output='screen',
        shell=True
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_activate_ndt', default_value='true'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odometry/local',
            description='Continuous local odometry topic used for NDT initial guesses',
        ),
        DeclareLaunchArgument(
            'align_initial_pose_stamp_to_cloud',
            default_value='true',
            description='Stamp NDT initial guesses with the latest live cloud stamp to avoid Autoware timestamp mismatch aborts.',
        ),
        DeclareLaunchArgument(
            'max_map_to_odom_translation_step',
            default_value='1.6',
            description='Maximum accepted map->odom correction step in meters for real JT128/NDT startup.',
        ),
        DeclareLaunchArgument(
            'max_map_to_odom_rotation_step_deg',
            default_value='45.0',
            description='Maximum accepted map->odom yaw correction step in degrees for real JT128/NDT startup.',
        ),
        DeclareLaunchArgument(
            'base_ndt_param_file',
            default_value=f'{ndt_share}/config/ndt_scan_matcher.param.yaml',
        ),
        DeclareLaunchArgument(
            'ndt_param_file',
            default_value=f'{a2_system_share}/config/ndt_scan_matcher_a2.yaml',
        ),
        ndt_node,
        adapter_node,
        auto_activate
    ])
