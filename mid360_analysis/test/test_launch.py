"""Launch configuration math and argument validation without a display or hardware."""
import importlib.util
import math
from pathlib import Path

import pytest
from launch import LaunchContext


def load(name):
    path = Path(__file__).resolve().parents[1] / 'launch' / (name + '.launch.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('matrix, expected', [
    ([1,0,0,0,1,0,0,0,1], [0,0,0,1]),
    ([1,0,0,0,-1,0,0,0,-1], [1,0,0,0]),
    ([-1,0,0,0,1,0,0,0,-1], [0,1,0,0]),
    ([-1,0,0,0,-1,0,0,0,1], [0,0,1,0]),
    ([0,-1,0,1,0,0,0,0,1], [0,0,math.sqrt(0.5),math.sqrt(0.5)]),
])
def test_static_transform_rotation(matrix, expected):
    assert load('static_tf').rotation_quaternion(matrix) == pytest.approx(expected)


@pytest.mark.parametrize('matrix', [[1]*9, [-1,0,0,0,1,0,0,0,1], [float('nan')]*9, [1,0]])
def test_rejects_invalid_rotation(matrix):
    with pytest.raises(ValueError):
        load('static_tf').rotation_quaternion(matrix)


def test_online_extrinsics_cannot_be_published_as_static(tmp_path):
    config = tmp_path / 'online.yaml'
    config.write_text('/**:\n  ros__parameters:\n    mapping:\n      extrinsic_est_en: true\n')
    context = LaunchContext()
    context.launch_configurations['fastlio_config'] = str(config)
    with pytest.raises(ValueError, match='extrinsic_est_en=false'):
        load('static_tf').start(context)


def test_replay_rejects_missing_bag():
    context = LaunchContext()
    context.launch_configurations['bag'] = '/does-not-exist/mid360_bag'
    with pytest.raises(RuntimeError, match='Bag does not exist'):
        load('analysis').playback(context)
