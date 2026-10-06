"""Real MID360 driver, supplied FAST-LIO, analysis and RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('mid360_analysis'))
    driver_config = LaunchConfiguration('user_config_path').perform(context)
    fastlio_config = Path(LaunchConfiguration('fastlio_config').perform(context)).resolve()
    for path in (Path(driver_config), fastlio_config):
        if not path.is_file():
            raise RuntimeError(f'Configuration does not exist: {path}')
    fast_share = Path(get_package_share_directory('fast_lio'))
    return [
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', parameters=[{
                 'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'output_data_type': 0, 'frame_id': 'livox_frame',
                 'user_config_path': driver_config, 'cmdline_input_bd_code': 'livox0000000001',
             }]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fast_share / 'launch/mapping.launch.py')),
            launch_arguments={'config_path': str(fastlio_config.parent),
                              'config_file': fastlio_config.name, 'rviz': 'false',
                              'use_sim_time': 'false'}.items())]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/static_tf.launch.py')),
            condition=IfCondition(LaunchConfiguration('with_static_tf')),
            launch_arguments={'fastlio_config': str(fastlio_config), 'use_sim_time': 'false'}.items()),
        Node(package='mid360_analysis', executable='imu_stats_node', name='imu_stats_node',
             output='screen', condition=IfCondition(LaunchConfiguration('with_analysis')),
             parameters=[LaunchConfiguration('params'), {'use_sim_time': False}]),
        Node(package='mid360_analysis', executable='odom_analyzer_node', name='odom_analyzer_node',
             output='screen', condition=IfCondition(LaunchConfiguration('with_analysis')),
             parameters=[LaunchConfiguration('params'), {'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/mid360_analysis.rviz')],
             parameters=[{'use_sim_time': False}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_analysis'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('with_analysis', default_value='true'),
        DeclareLaunchArgument('with_static_tf', default_value='true'),
        DeclareLaunchArgument('user_config_path', default_value=str(share / 'config/MID360_config.json'),
                              description='Driver JSON; set host and LiDAR IPs before starting'),
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/mid360.yaml')),
        DeclareLaunchArgument('params', default_value=str(share / 'config/analysis_params.yaml')),
        OpaqueFunction(function=start),
    ])
