"""Analyze recorded FAST-LIO output. Never starts the driver or FAST-LIO."""
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def playback(context):
    bag = LaunchConfiguration('bag').perform(context)
    if not bag:
        return []
    if not Path(bag).exists():
        raise RuntimeError(f'Bag does not exist: {bag}')
    rate = float(LaunchConfiguration('rate').perform(context))
    delay = float(LaunchConfiguration('play_delay_s').perform(context))
    if not math.isfinite(rate) or rate <= 0 or not math.isfinite(delay) or delay < 0:
        raise RuntimeError('rate must be positive and play_delay_s nonnegative')
    share = Path(get_package_share_directory('mid360_analysis'))
    return [ExecuteProcess(cmd=[
        'ros2', 'bag', 'play', bag, '--clock', '--rate', str(rate), '--delay', str(delay),
        '--disable-keyboard-controls', '--qos-profile-overrides-path', str(share / 'config/bag_qos.yaml'),
        '--topics', '/livox/lidar', '/livox/imu', '/Odometry', '/path', '/cloud_registered',
        '/cloud_registered_body', '/Laser_map', '/tf', '/tf_static',
    ], output='screen')]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_analysis'))
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='', description='Empty: start analysis and play the bag separately'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('play_delay_s', default_value='2.0', description='DDS discovery delay before playback'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params', default_value=str(share / 'config/analysis_params.yaml')),
        DeclareLaunchArgument('with_static_tf', default_value='false', description='Enable only if bag lacks body -> livox_frame'),
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/mid360.yaml')),
        Node(package='mid360_analysis', executable='imu_stats_node', name='imu_stats_node', output='screen',
             parameters=[LaunchConfiguration('params'), {'use_sim_time': True}]),
        Node(package='mid360_analysis', executable='odom_analyzer_node', name='odom_analyzer_node', output='screen',
             parameters=[LaunchConfiguration('params'), {'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/static_tf.launch.py')),
            condition=IfCondition(LaunchConfiguration('with_static_tf')),
            launch_arguments={'fastlio_config': LaunchConfiguration('fastlio_config'), 'use_sim_time': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/mid360_analysis.rviz')],
             parameters=[{'use_sim_time': True}]),
        OpaqueFunction(function=playback),
    ])
