"""Transport launch regression checks without creating nodes or DDS participants."""

import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from kv_executive.launch_support import fastdds_udp_environment
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
ENVIRONMENT_KEYS = (
    'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
    'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
)


@pytest.fixture
def context(monkeypatch, tmp_path):
    """Use a known environment and let pytest restore every inherited setting."""
    for key in ENVIRONMENT_KEYS:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    result = LaunchContext()
    result.launch_configurations['udp_only'] = 'true'
    return result


def apply(actions, context):
    """Execute only in-process launch actions; no process actions are accepted."""
    for action in actions:
        assert not isinstance(action, Node)
        if isinstance(action, LogInfo):
            continue
        children = action.execute(context)
        if children:
            apply(children, context)


def launch_module(filename):
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), PACKAGE / 'launch' / filename,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('rmw', [None, '', 'rmw_fastrtps_cpp'])
def test_default_and_fastdds_select_udp(context, monkeypatch, rmw):
    if rmw is not None:
        monkeypatch.setenv('RMW_IMPLEMENTATION', rmw)
    before = dict(os.environ)
    actions = fastdds_udp_environment(context, 'udp_only', 'Test monitor')
    assert dict(os.environ) == before
    apply(actions, context)
    profile = context.environment['FASTRTPS_DEFAULT_PROFILES_FILE']
    assert profile.endswith('/kv_executive/config/fastdds_udp.xml')
    assert context.environment['FASTDDS_DEFAULT_PROFILES_FILE'] == profile
    assert context.environment.get('RMW_IMPLEMENTATION') == rmw


@pytest.mark.parametrize('rmw', [
    'rmw_cyclonedds_cpp', 'rmw_fastrtps_dynamic_cpp', 'rmw_connextdds',
])
def test_other_rmw_preserves_environment(context, monkeypatch, rmw):
    monkeypatch.setenv('RMW_IMPLEMENTATION', rmw)
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/custom/profile.xml')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    before = dict(os.environ)
    assert fastdds_udp_environment(context, 'udp_only', 'Test monitor') == []
    assert dict(os.environ) == before


@pytest.mark.parametrize('disabled', ['false', 'False', '0'])
def test_opt_out_preserves_custom_profiles(context, monkeypatch, disabled):
    context.launch_configurations['udp_only'] = disabled
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/custom/old.xml')
    monkeypatch.setenv('FASTDDS_DEFAULT_PROFILES_FILE', '/custom/new.xml')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    before = dict(os.environ)
    assert fastdds_udp_environment(context, 'udp_only', 'Test monitor') == []
    assert dict(os.environ) == before


@pytest.mark.parametrize('discovery,legacy,local_only', [
    ('', '', False), ('SYSTEM_DEFAULT', '', False), ('SUBNET', '0', False),
    ('OFF', '', False), ('LOCALHOST', '', True), ('invalid', '', True),
    ('localhost', '', True), ('', '1', True), ('SUBNET', '1', True),
    ('OFF', '1', True), ('', '10', True),
])
def test_discovery_scope_is_preserved(context, monkeypatch, discovery, legacy, local_only):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', discovery)
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', legacy)
    apply(fastdds_udp_environment(context, 'udp_only', 'Test monitor'), context)
    filename = 'fastdds_udp_localhost.xml' if local_only else 'fastdds_udp.xml'
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'].endswith(filename)
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == (
        'SYSTEM_DEFAULT' if local_only else discovery
    )
    assert context.environment['ROS_LOCALHOST_ONLY'] == (
        '0' if legacy.startswith('1') else legacy
    )


@pytest.mark.parametrize('switch', ['true', 'false'])
@pytest.mark.parametrize('local_only', [False, True])
def test_group_restores_environment_for_battery_and_other_nodes(
    context, monkeypatch, switch, local_only,
):
    context.launch_configurations['udp_only'] = switch
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/custom/profile.xml')
    if local_only:
        monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    before = dict(os.environ)
    observed = []

    def capture(context):
        observed.append(dict(context.environment))
        return []

    group = GroupAction(actions=[
        OpaqueFunction(
            function=fastdds_udp_environment, args=['udp_only', 'Detection monitor'],
        ),
        OpaqueFunction(function=capture),
    ])
    apply([group], context)
    assert dict(os.environ) == before
    assert len(observed) == 1
    if switch == 'true':
        assert observed[0]['FASTRTPS_DEFAULT_PROFILES_FILE'] != '/custom/profile.xml'
    else:
        assert observed[0] == before


@pytest.mark.parametrize('filename', [
    'autonomy.launch.py', 'monitoring.launch.py', 'detection_monitor.launch.py', 'px4.launch.py',
])
def test_drone_default_matches_kv_drone(context, filename):
    description = launch_module(filename).generate_launch_description()
    arguments = [
        action for action in description.entities if isinstance(action, DeclareLaunchArgument)
    ]
    drone = next(action for action in arguments if action.name == 'drone_name')
    assert perform_substitutions(context, drone.default_value) == 'kv_drone'


def test_monitoring_forwards_transport_switch_only_to_detection(context):
    description = launch_module('monitoring.launch.py').generate_launch_description()
    context.launch_configurations['detection_monitor_udp_only'] = 'false'
    includes = [
        action for action in description.entities if isinstance(action, IncludeLaunchDescription)
    ]
    battery_arguments = dict(includes[0].launch_arguments)
    detection_arguments = dict(includes[1].launch_arguments)
    assert 'detection_monitor_udp_only' not in battery_arguments
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            detection_arguments['detection_monitor_udp_only'],
        ),
    ) == 'false'


def test_detection_transport_actions_are_scoped(context):
    description = launch_module('detection_monitor.launch.py').generate_launch_description()
    assert not any(isinstance(action, Node) for action in description.entities)
    group = next(action for action in description.entities if isinstance(action, GroupAction))
    children = group.get_sub_entities()
    assert children[0].__class__.__name__ == 'PushLaunchConfigurations'
    assert children[1].__class__.__name__ == 'PushEnvironment'
    assert children[-2].__class__.__name__ == 'PopEnvironment'
    assert any(isinstance(action, OpaqueFunction) for action in children)
    assert sum(isinstance(action, Node) for action in children) == 1


@pytest.mark.parametrize('filename', ['fastdds_udp.xml', 'fastdds_udp_localhost.xml'])
def test_udp_profiles_disable_builtin_transports_and_keep_qos(filename):
    profile = ET.parse(PACKAGE / 'config' / filename).getroot()
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert profile.find('.//dds:useBuiltinTransports', ns).text == 'false'
    assert profile.find('.//dds:transport_descriptor/dds:type', ns).text == 'UDPv4'
    assert profile.find('.//dds:publisher', ns) is None
    assert profile.find('.//dds:subscriber', ns) is None
    addresses = [node.text for node in profile.findall('.//dds:address', ns)]
    assert addresses == (['127.0.0.1'] * 3 if 'localhost' in filename else [])
