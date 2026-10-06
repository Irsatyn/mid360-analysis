"""Real MID360 driver, supplied FAST-LIO, analysis and RViz."""
from pathlib import Path
import copy
import ipaddress
import json
import os
import socket
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def unicast_ipv4(value):
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_loopback or address.is_reserved:
        raise ValueError(f'Expected a unicast IPv4 address, got {value!r}')
    return str(address)


def route_host_ip(lidar_ip):
    # UDP connect selects a route/source address without sending a packet.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            connection.connect((lidar_ip, 56100))
            return unicast_ipv4(connection.getsockname()[0])
    except OSError as error:
        raise ValueError('Cannot select host IP for LiDAR; check routing or set host_ip explicitly') from error


def configure_driver_network(config, lidar_ip='', host_ip=''):
    updated = copy.deepcopy(config)
    lidars = updated.get('lidar_configs', [])
    if not lidars:
        raise ValueError('Driver JSON must contain lidar_configs')
    if lidar_ip:
        if len(lidars) != 1:
            raise ValueError('lidar_ip override requires a single LiDAR; configure multiple devices in JSON')
        lidars[0]['ip'] = unicast_ipv4(lidar_ip)
    for lidar in lidars:
        if not lidar.get('ip'):
            raise ValueError('Set lidar_ip:=<device IPv4> or provide a configured driver JSON')
        lidar['ip'] = unicast_ipv4(lidar['ip'])
    try:
        host = updated['MID360']['host_net_info']
    except KeyError as error:
        raise ValueError('Driver JSON must contain MID360.host_net_info') from error
    fields = ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')
    selected = None
    if host_ip or any(not host.get(key) for key in fields):
        selected = route_host_ip(lidars[0]['ip']) if host_ip in ('', 'auto') else unicast_ipv4(host_ip)
    for key in fields:
        value = selected if host_ip or not host.get(key) else host[key]
        host[key] = unicast_ipv4(value)
        if host[key] in [lidar['ip'] for lidar in lidars]:
            raise ValueError('Host and LiDAR must have different IP addresses')
    if host.get('log_data_ip'):
        host['log_data_ip'] = unicast_ipv4(selected if host_ip else host['log_data_ip'])
    return updated


def prepare_driver_config(context):
    source = Path(LaunchConfiguration('user_config_path').perform(context))
    config = configure_driver_network(json.loads(source.read_text()),
                                      context.launch_configurations.get('lidar_ip', ''),
                                      context.launch_configurations.get('host_ip', ''))
    descriptor, path = tempfile.mkstemp(prefix='mid360_driver_', suffix='.json')
    with os.fdopen(descriptor, 'w') as stream:
        json.dump(config, stream, indent=2)
    context.launch_configurations['_mid360_runtime_driver_config'] = path
    return path


def cleanup_driver_config(context):
    path = context.launch_configurations.pop('_mid360_runtime_driver_config', None)
    if path:
        Path(path).unlink(missing_ok=True)
    return []


def start(context):
    share = Path(get_package_share_directory('mid360_analysis'))
    driver_config = LaunchConfiguration('user_config_path').perform(context)
    fastlio_config = Path(LaunchConfiguration('fastlio_config').perform(context)).resolve()
    for path in (Path(driver_config), fastlio_config):
        if not path.is_file():
            raise RuntimeError(f'Configuration does not exist: {path}')
    fast_share = Path(get_package_share_directory('fast_lio'))
    driver_config = prepare_driver_config(context)
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
                              description='Driver JSON template; input file is never overwritten'),
        DeclareLaunchArgument('lidar_ip', default_value='', description='Device IPv4; empty uses driver JSON'),
        DeclareLaunchArgument('host_ip', default_value='',
                              description='Host IPv4 or auto; empty preserves JSON addresses and auto-fills missing ones'),
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/mid360.yaml')),
        DeclareLaunchArgument('params', default_value=str(share / 'config/analysis_params.yaml')),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_driver_config)])),
        OpaqueFunction(function=start),
    ])
