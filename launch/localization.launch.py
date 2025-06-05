from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for AGV localization system"""
    
    # Get the package share directory
    pkg_dir = get_package_share_directory('agribot')
    
    # EKF config file path
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf_config.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gps = LaunchConfiguration('use_gps')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_use_gps = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS for localization if true'
    )
    
    # GPS to Odometry node (only if GPS is enabled)
    gps_to_odom_node = Node(
        package='agribot',
        executable='gps_to_odom',
        name='gps_to_odom',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'utm_zone': 48},  # Default for Vietnam
            {'northern_hemisphere': True},
            {'gps_noise_xy': 1.0}
        ],
        output='screen',
        condition=IfCondition(use_gps)
    )
    
    # Robot localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            ekf_config_path
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # AGV Localization node (simple heading processing)
    agv_localization_node = Node(
        package='agribot',
        executable='localization',
        name='agv_localization',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_gps,
        gps_to_odom_node,
        ekf_node,
        agv_localization_node
    ]) 