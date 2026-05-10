from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ndt_share = get_package_share_directory('autoware_ndt_scan_matcher')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ndt_param_file = LaunchConfiguration('ndt_param_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'ndt_param_file',
            default_value=f'{ndt_share}/config/ndt_scan_matcher.param.yaml',
        ),

        # NDT Scan Matcher Node
        Node(
            package='autoware_ndt_scan_matcher',
            executable='autoware_ndt_scan_matcher_node',
            name='ndt_scan_matcher',
            parameters=[
                ndt_param_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('points_raw', '/jt128/front/points'),
            ],
            output='screen'
        ),
        
        # A2 NDT Adapter Node
        Node(
            package='a2_ndt_adapter',
            executable='ndt_adapter_node',
            name='ndt_adapter',
            parameters=[{
                'use_sim_time': use_sim_time,
                'live_cloud_topic': '/jt128/front/points',
                'odom_topic': '/jt128/dlio/odom',
                'map_topic': '/a2/map/pointcloud_3d',
                'pose_topic': '/a2/relocalization/pose',
                'status_topic': '/a2/relocalization/status',
                'score_topic': 'nearest_voxel_transformation_likelihood',
                'score_threshold': 2.3,
                'score_min_is_good': False,
                'ndt_initial_pose_topic': '/a2/ndt/adapter_ignored_initial_pose',
            }],
            output='screen'
        )
    ])
