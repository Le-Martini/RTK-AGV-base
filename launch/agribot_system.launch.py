from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Khai báo các tham số
    debug = LaunchConfiguration('debug')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gps = LaunchConfiguration('use_gps')
    
    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS for localization if true'
    )

    # Đường dẫn đến file HTML
    html_path = os.path.join(
        get_package_share_directory('agribot'),
        'web',
        'index.html'
    )

    # Đường dẫn đến file cấu hình EKF
    ekf_config_path = os.path.join(
        get_package_share_directory('agribot'),
        'config',
        'ekf_config.yaml'
    )

    # Node rosbridge_server
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )

    # Node để serve file HTML
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=os.path.dirname(html_path),
        output='screen'
    )

    # Các node của agribot
    esp32_node = Node(
        package='agribot',
        executable='esp32_node',
        name='esp32_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    gps_node = Node(
        package='agribot',
        executable='gps_node',
        name='gps_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_gps)
    )

    imu_node = Node(
        package='agribot',
        executable='imu_node',
        name='imu_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # GPS to Odometry node
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

    # AGV Localization node (xử lý heading và kết hợp dữ liệu)
    localization_node = Node(
        package='agribot',
        executable='localization',
        name='agv_localization',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    main_control = Node(
        package='agribot',
        executable='main_control',
        name='main_control',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        debug_arg,
        use_sim_time_arg,
        use_gps_arg,
        rosbridge_node,
        http_server,
        esp32_node,
        gps_node,
        imu_node,
        gps_to_odom_node,
        ekf_node,
        localization_node,
        main_control
    ])