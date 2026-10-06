"""Create a synthetic bag and verify the installed analysis Launch using /clock."""
import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Float64MultiArray
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message


def write_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, kind in [('/livox/imu','sensor_msgs/msg/Imu'),
                       ('/Odometry','nav_msgs/msg/Odometry'), ('/tf','tf2_msgs/msg/TFMessage')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
    events = []
    for index in range(250):
        stamp = 100_000_000_000 + index*5_000_000
        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = 'livox_frame'
        msg.linear_acceleration.z = 1.
        events.append((stamp, '/livox/imu', msg))
    for index in range(13):
        stamp = 100_000_000_000 + index*100_000_000
        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = 'camera_init'; msg.child_frame_id = 'body'
        msg.pose.pose.position.x = index*0.01; msg.pose.pose.orientation.w = 1.
        events.append((stamp, '/Odometry', msg))
        tf = TransformStamped()
        tf.header = msg.header; tf.child_frame_id = 'body'
        tf.transform.translation.x = index*0.01; tf.transform.rotation.w = 1.
        events.append((stamp+10_000_000, '/tf', TFMessage(transforms=[tf])))
    for stamp, topic, msg in sorted(events, key=lambda event: event[0]):
        writer.write(topic, serialize_message(msg), stamp)
    del writer  # finalize metadata before launching player


def test_analysis_launch_replays_bag_with_simulated_time(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(50 + os.getpid() % 150)
    bag = tmp_path / 'synthetic_bag'
    write_bag(bag)
    params = tmp_path / 'params.yaml'
    params.write_text('imu_stats_node:\n  ros__parameters:\n    window_size: 5\n    publish_every_n: 1\n')
    rclpy.init()
    node = rclpy.create_node('bag_replay_probe')
    data = {}
    subscriptions = [node.create_subscription(kind, name,
                     lambda msg, key=name: data.setdefault(key, []).append(msg), 100)
                     for name, kind in [('/analysis/imu/is_static',Bool), ('/analysis/imu/rate',Float64),
                     ('/analysis/imu/stats',Float64MultiArray), ('/analysis/odom/distance',Float64),
                     ('/analysis/odom/speed',Float64), ('/analysis/odom/tf_error',Float64MultiArray)]]
    log_path = tmp_path / 'launch.log'
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2','launch','mid360_analysis','analysis.launch.py', 'rviz:=false',
                f'bag:={bag}', f'params:={params}', 'play_delay_s:=2.0'],
                stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic()+15.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if data.get('/analysis/odom/distance') and data['/analysis/odom/distance'][-1].data >= 0.119:
                    # Allow the last delayed TF and IMU samples to be consumed.
                    end = time.monotonic()+0.3
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    break
                assert process.poll() is None, log_path.read_text()
        assert data.get('/analysis/imu/is_static'), log_path.read_text()
        assert data['/analysis/imu/is_static'][-1].data
        assert data['/analysis/imu/rate'][-1].data == pytest.approx(200.)
        assert math.isnan(data['/analysis/imu/stats'][-1].data[16])
        assert data['/analysis/odom/distance'][-1].data == pytest.approx(0.12)
        assert data['/analysis/odom/speed'][-1].data == pytest.approx(0.1)
        assert data['/analysis/odom/tf_error'][-1].data == pytest.approx([0.,0.,0.])
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node(); rclpy.shutdown()
