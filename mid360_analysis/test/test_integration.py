"""Hardware-free tests of the actual C++ executables over DDS."""
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Float64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray
from rclpy.qos import qos_profile_sensor_data


@pytest.fixture
def system(request):
    node_dir = Path(os.environ.get('MID360_NODE_DIR', 'build/mid360_analysis'))
    executables = [node_dir / name for name in ('imu_stats_node', 'odom_analyzer_node')]
    assert all(p.is_file() for p in executables), 'C++ analysis executables have not been built'
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 150)
    rclpy.init()
    node = rclpy.create_node('analysis_integration_probe')
    received = {}
    topics = {
        '/analysis/imu/rate': Float64, '/analysis/imu/dt_jitter': Float64,
        '/analysis/imu/acc_norm': Float64, '/analysis/imu/is_static': Bool,
        '/analysis/imu/gyro_bias': Vector3Stamped, '/analysis/imu/stats': Float64MultiArray,
        '/analysis/odom/distance': Float64, '/analysis/odom/speed': Float64,
        '/analysis/odom/displacement': Float64, '/analysis/odom/rpy': Vector3Stamped,
        '/analysis/odom/velocity': TwistStamped, '/analysis/odom/tf_error': Float64MultiArray,
        '/analysis/odom/markers': MarkerArray,
    }
    subscriptions = [node.create_subscription(
        kind, topic, lambda msg, t=topic: received.setdefault(t, []).append(msg), 20)
        for topic, kind in topics.items()]
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/Odometry', qos_profile_sensor_data)
    tf_pub = TransformBroadcaster(node)
    logs = [open(request.node.name + '_' + name.name + '.log', 'w+') for name in executables]
    unit = getattr(request, 'param', True)
    received["_acc_unit_is_g"] = unit
    arguments = [
        ['-p', 'window_size:=5', '-p', 'publish_every_n:=1', '-p', f'acc_unit_is_g:={str(unit).lower()}'],
        ['-p', 'tf_wait_s:=0.15', '-p', 'marker_every_m:=0.01', '-p', 'max_trajectory_points:=20',
         '-p', 'max_distance_markers:=3'],
    ]
    processes = []
    try:
        for exe, args, log in zip(executables, arguments, logs):
            processes.append(subprocess.Popen([str(exe), '--ros-args', *args], stdout=log, stderr=log))
        def spin(seconds=0.1):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                assert all(p.poll() is None for p in processes), 'analysis node exited unexpectedly'
                rclpy.spin_once(node, timeout_sec=0.01)
        deadline = time.monotonic()+8.
        while imu_pub.get_subscription_count() == 0 or odom_pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'input subscriptions not discovered'
            spin()
        spin(0.4)  # allow output discovery too
        yield received, imu_pub, odom_pub, tf_pub, spin
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait()
        for log in logs:
            log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def imu(index, scale=1.):
    msg = Imu()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = index * 5_000_000
    msg.header.frame_id = 'livox_frame'
    msg.linear_acceleration.z = scale
    msg.angular_velocity.x = 0.001
    return msg


def odom(index, x=0.):
    msg = Odometry()
    msg.header.stamp.sec = 100 + index // 10
    msg.header.stamp.nanosec = (index % 10) * 100_000_000
    msg.header.frame_id = 'camera_init'
    msg.child_frame_id = 'body'
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.
    return msg


def transform(msg):
    tf = TransformStamped()
    tf.header = msg.header
    tf.child_frame_id = msg.child_frame_id
    tf.transform.translation.x = msg.pose.pose.position.x
    tf.transform.rotation.w = -1.  # same rotation, opposite quaternion sign
    return tf


@pytest.mark.parametrize('system', [True, False], indirect=True)
def test_static_units_and_all_imu_outputs(system):
    data, pub, _, _, spin = system
    scale = 1. if data['_acc_unit_is_g'] else 9.80665
    for i in range(8):
        pub.publish(imu(i, scale)); spin(0.025)
    spin(0.1)
    assert data['/analysis/imu/is_static'][-1].data
    assert data['/analysis/imu/acc_norm'][-1].data == pytest.approx(1.)
    assert data['/analysis/imu/rate'][-1].data == pytest.approx(200.)
    assert data['/analysis/imu/dt_jitter'][-1].data == pytest.approx(0., abs=1e-12)
    assert data['/analysis/imu/gyro_bias'][-1].vector.x == pytest.approx(0.001)
    assert len(data['/analysis/imu/stats'][-1].data) == 17


def test_slow_motion_velocity_markers_and_bounds(system):
    data, _, pub, tf_pub, spin = system
    for i in range(45):
        msg = odom(i, 0.003*i)
        tf_pub.sendTransform(transform(msg)); pub.publish(msg); spin(0.02)
    spin(0.2)
    assert data['/analysis/odom/distance'][-1].data == pytest.approx(0.132)
    assert data['/analysis/odom/speed'][-1].data == pytest.approx(0.03)
    assert data['/analysis/odom/velocity'][-1].twist.linear.x == pytest.approx(0.03)
    assert data['/analysis/odom/displacement'][-1].data == pytest.approx(0.132)
    assert data['/analysis/odom/rpy'][-1].vector.z == pytest.approx(0.)
    markers = data['/analysis/odom/markers'][-1].markers
    line = next(m for m in markers if m.ns == 'trajectory')
    assert len(line.points) <= 20
    assert len(line.points) == len(line.colors)
    assert sum(m.ns == 'distance' and m.action == m.ADD for m in markers) <= 3


def test_delayed_tf_and_antipodal_quaternion(system):
    data, _, pub, tf_pub, spin = system
    msg = odom(0, 2.)
    pub.publish(msg); spin(0.06)
    tf_pub.sendTransform(transform(msg)); spin(0.2)
    error = data['/analysis/odom/tf_error'][-1].data
    assert error == pytest.approx([0., 0., 0.])


def test_missing_tf_reports_nan_and_count(system):
    import math
    data, _, pub, _, spin = system
    pub.publish(odom(0)); spin(0.35)
    error = data['/analysis/odom/tf_error'][-1].data
    assert math.isnan(error[0]) and math.isnan(error[1])
    assert error[2] == 1.


def test_backward_time_resets_distance_and_deletes_old_markers(system):
    data, _, pub, tf_pub, spin = system
    for i in (0, 1, 2):
        msg = odom(i, float(i)); tf_pub.sendTransform(transform(msg)); pub.publish(msg); spin()
    assert data['/analysis/odom/distance'][-1].data == pytest.approx(2.)
    msg = odom(0, 10.); pub.publish(msg); spin()
    assert data['/analysis/odom/distance'][-1].data == 0.
    assert any(m.action == m.DELETEALL for m in data['/analysis/odom/markers'][-1].markers)
