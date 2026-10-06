"""Launch configuration math and argument validation without a display or hardware."""
import importlib.util
import math
import json
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
        'lidar_ip': '192.0.2.30', 'host_ip': '192.0.2.50',
    })
    actions = module.start(context)
    visit_all_entities_and_collect_futures(actions[1], context)
    assert actions[-1].condition.evaluate(context), 'FAST-LIO include must preserve outer RViz selection'
    module.cleanup_driver_config(context)


def network_template():
    config = json.loads((Path(__file__).resolve().parents[1] / 'config/MID360_config.json').read_text())
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        config['MID360']['host_net_info'][key] = ''
    config['lidar_configs'][0]['ip'] = ''
    return config


def test_explicit_addresses_override_template_without_changing_ports_or_source():
    module = load('bringup')
    config = network_template()
    updated = module.configure_driver_network(config, '10.42.0.30', '10.42.0.50')
    host = updated['MID360']['host_net_info']
    assert [host[k] for k in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')] == ['10.42.0.50'] * 4
    assert host['point_data_port'] == 56301
    assert host['log_data_ip'] == ''
    assert updated['lidar_configs'][0]['ip'] == '10.42.0.30'
    assert config['lidar_configs'][0]['ip'] == ''


def test_auto_host_uses_route_to_lidar(monkeypatch):
    module = load('bringup')
    monkeypatch.setattr(module, 'route_host_ip', lambda ip: '10.42.0.50' if ip == '10.42.0.30' else pytest.fail(ip))
    updated = module.configure_driver_network(network_template(), '10.42.0.30')
    assert updated['MID360']['host_net_info']['imu_data_ip'] == '10.42.0.50'


def test_complete_custom_network_is_preserved_without_route_lookup(monkeypatch):
    module = load('bringup')
    config = network_template()
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        config['MID360']['host_net_info'][key] = '10.99.0.50'
    config['lidar_configs'][0]['ip'] = '10.99.0.30'
    monkeypatch.setattr(module, 'route_host_ip', lambda _: pytest.fail('custom host must be preserved'))
    assert module.configure_driver_network(config) == config


@pytest.mark.parametrize('lidar, host', [('', '10.42.0.50'), ('bad-ip', '10.42.0.50'),
                                      ('10.42.0.30', '0.0.0.0'), ('10.42.0.30', '127.0.0.1'),
                                      ('224.0.0.1', '10.42.0.50'), ('10.42.0.30', '::1')])
def test_invalid_network_addresses_fail_before_start(lidar, host):
    with pytest.raises(ValueError):
        load('bringup').configure_driver_network(network_template(), lidar, host)


def test_runtime_config_is_separate_and_removed_on_shutdown(tmp_path):
    module = load('bringup')
    source = tmp_path / 'source.json'
    original = json.dumps(network_template())
    source.write_text(original)
    context = LaunchContext()
    context.launch_configurations.update(user_config_path=str(source), lidar_ip='10.42.0.30', host_ip='10.42.0.50')
    runtime = Path(module.prepare_driver_config(context))
    try:
        assert runtime != source
        assert json.loads(runtime.read_text())['lidar_configs'][0]['ip'] == '10.42.0.30'
        assert source.read_text() == original
    finally:
        module.cleanup_driver_config(context)
    assert not runtime.exists()
    assert source.exists()


def test_dds_generator_supports_interface_and_unicast_peers(tmp_path):
    import subprocess
    import sys
    import xml.etree.ElementTree as ET
    tool = Path(__file__).resolve().parents[2] / 'tools/configure_dds.py'
    output = tmp_path / 'dds.xml'
    result = subprocess.run([sys.executable, str(tool), '--output', str(output),
                             '--interface', 'eno1', '--multicast', 'false', '--peer', '10.42.0.60'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    root = ET.parse(output).getroot()
    ns = {'c': 'https://cdds.io/config'}
    assert root.find('.//c:NetworkInterface', ns).attrib == {'name': 'eno1'}
    assert root.find('.//c:AllowMulticast', ns).text == 'false'
    assert {'Address': '10.42.0.60'} in [peer.attrib for peer in root.findall('.//c:Peer', ns)]


@pytest.mark.parametrize('interface, expected', [('auto', {'autodetermine': 'true'}),
                                                ('10.42.0.50', {'address': '10.42.0.50'})])
def test_dds_generator_supports_auto_and_ip_selection(tmp_path, interface, expected):
    import subprocess
    import sys
    import xml.etree.ElementTree as ET
    tool = Path(__file__).resolve().parents[2] / 'tools/configure_dds.py'
    output = tmp_path / 'dds.xml'
    subprocess.run([sys.executable, str(tool), '--output', str(output), '--interface', interface], check=True)
    assert ET.parse(output).find('.//{https://cdds.io/config}NetworkInterface').attrib == expected


def test_dds_generator_rejects_multicast_peer_without_writing(tmp_path):
    import subprocess
    import sys
    tool = Path(__file__).resolve().parents[2] / 'tools/configure_dds.py'
    output = tmp_path / 'dds.xml'
    result = subprocess.run([sys.executable, str(tool), '--output', str(output), '--peer', '224.0.0.1'],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert not output.exists()
