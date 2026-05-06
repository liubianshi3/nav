import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    a2_ndt_adapter_share = get_package_share_directory('a2_ndt_adapter')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ndt_param_file = LaunchConfiguration('ndt_param_file', 
        default='/opt/ros/humble/share/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ndt_param_file', 
            default_value='/opt/ros/humble/share/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml'),
        
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
                # ekf_pose_with_covariance is used as is, connected to adapter
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
            }],
            output='screen'
        )
    ])
