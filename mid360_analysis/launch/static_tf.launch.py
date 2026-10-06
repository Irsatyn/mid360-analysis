"""Derive body -> LiDAR static TF from the same fixed extrinsics as FAST-LIO."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def rotation_quaternion(r):
    if len(r) != 9 or not all(math.isfinite(v) for v in r):
        raise ValueError('extrinsic_R requires nine finite values')
    for i in range(3):
        for j in range(3):
            dot = sum(r[3*k+i]*r[3*k+j] for k in range(3))
            if abs(dot-(1. if i == j else 0.)) > 1e-5:
                raise ValueError('extrinsic_R must be orthonormal')
    det = r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6])
    if abs(det-1.) > 1e-5:
        raise ValueError('extrinsic_R must have determinant +1')
    trace = r[0]+r[4]+r[8]
    if trace > 0:
        s = 2.*math.sqrt(trace+1.)
        return ((r[7]-r[5])/s, (r[2]-r[6])/s, (r[3]-r[1])/s, s/4.)
    i = max(range(3), key=lambda k: r[3*k+k])
    j, k = (i+1) % 3, (i+2) % 3
    s = 2.*math.sqrt(1.+r[3*i+i]-r[3*j+j]-r[3*k+k])
    q = [0., 0., 0., 0.]
    q[i] = s/4.; q[j] = (r[3*j+i]+r[3*i+j])/s; q[k] = (r[3*k+i]+r[3*i+k])/s
    q[3] = (r[3*k+j]-r[3*j+k])/s
    return tuple(q)


def start(context):
    path = LaunchConfiguration('fastlio_config').perform(context)
    with open(path, encoding='utf-8') as file:
        mapping = yaml.safe_load(file)['/**']['ros__parameters']['mapping']
    if mapping.get('extrinsic_est_en', False):
        raise ValueError('Static TF requires extrinsic_est_en=false; disable with_static_tf for online estimation')
    t = mapping['extrinsic_T']
    if len(t) != 3 or not all(math.isfinite(v) for v in t):
        raise ValueError('extrinsic_T requires three finite values')
    q = rotation_quaternion(mapping['extrinsic_R'])
    return [Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
                 arguments=['--x', str(t[0]), '--y', str(t[1]), '--z', str(t[2]),
                            '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                            '--frame-id', 'body', '--child-frame-id', 'livox_frame'],
                 parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_analysis'))
    return LaunchDescription([
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/mid360.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start),
    ])
