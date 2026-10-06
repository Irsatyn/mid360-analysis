#!/usr/bin/env python3
"""Opt-in hardware regression: receive both streams, then require clean exit.

Source ROS and the driver workspace first. Logs may contain device identifiers;
use a directory under bags/ to keep them out of Git.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--log-dir', required=True, type=Path)
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--seconds', type=float, default=8.0)
    args = parser.parse_args()
    if args.cycles < 1 or args.seconds < 4:
        parser.error('cycles must be positive and seconds must be at least 4')
    if not args.driver.is_file() or not args.config.is_file():
        parser.error('driver and config must be existing files')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg

    args.log_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('driver_shutdown_regression')
    counts = {'imu': 0, 'lidar': 0}

    def count(kind):
        def callback(_):
            counts[kind] += 1
        return callback

    subscriptions = [
        node.create_subscription(Imu, '/livox/imu', count('imu'), qos_profile_sensor_data),
        node.create_subscription(CustomMsg, '/livox/lidar', count('lidar'), qos_profile_sensor_data),
    ]
    results = []
    try:
        for cycle in range(1, args.cycles + 1):
            counts.update(imu=0, lidar=0)
            with (args.log_dir / f'cycle_{cycle}.log').open('w') as log:
                process = subprocess.Popen([
                    str(args.driver.resolve()), '--ros-args',
                    '-p', 'xfer_format:=1', '-p', 'output_data_type:=0',
                    '-p', 'publish_freq:=10.0', '-p', 'frame_id:=livox_frame',
                    '-p', 'user_config_path:=' + str(args.config.resolve()),
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                forced = False
                try:
                    end = time.monotonic() + args.seconds
                    while time.monotonic() < end and process.poll() is None:
                        rclpy.spin_once(node, timeout_sec=0.05)
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        code = process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        forced = True
                        os.killpg(process.pid, signal.SIGKILL)
                        code = process.wait()
                result = {'cycle': cycle, **counts, 'exit_code': code, 'forced': forced}
                result['passed'] = code == 0 and not forced and counts['imu'] > 0 and counts['lidar'] > 0
                results.append(result)
                print(json.dumps(result), flush=True)
            # Drain late best-effort samples before the next process starts.
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (args.log_dir / 'results.json').write_text(json.dumps(results, indent=2))
    return 0 if results and all(result['passed'] for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
