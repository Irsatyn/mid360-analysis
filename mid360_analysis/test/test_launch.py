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


def test_bringup_preserves_outer_rviz_argument(tmp_path, monkeypatch):
    from launch.utilities import visit_all_entities_and_collect_futures
    module = load('bringup')
    fake = tmp_path / 'fast_lio'
    (fake / 'launch').mkdir(parents=True)
    (fake / 'launch/mapping.launch.py').write_text(
        'from launch import LaunchDescription\ndef generate_launch_description():\n    return LaunchDescription([])\n')
    share = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: str(fake if package == 'fast_lio' else share))
    context = LaunchContext()
    context.launch_configurations.update({
        'user_config_path': str(share / 'config/MID360_config.json'),
        'fastlio_config': str(share / 'config/mid360.yaml'), 'rviz': 'true',
        'with_static_tf': 'false', 'with_analysis': 'true',
        'params': str(share / 'config/analysis_params.yaml'),
    })
    actions = module.start(context)
    visit_all_entities_and_collect_futures(actions[1], context)
    assert actions[-1].condition.evaluate(context), 'FAST-LIO include must preserve outer RViz selection'
