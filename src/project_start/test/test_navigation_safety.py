"""ROS-independent regressions for safety gates, WGS84 input, and shipped config."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from project_start.safety import fresh, variance_ok, validate_route, check_legs

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('receipt,stamp,expected', [
    (0.1, 0.1, True), (0.1, -0.05, True), (0.5, 0.1, False),
    (0.1, 0.5, False), (-1, 0.1, False), (0.1, -1, False),
    (math.nan, 0.1, False), (0.1, math.inf, False)])
def test_fresh(receipt, stamp, expected):
    assert fresh(receipt, stamp, 0.4) == expected


@pytest.mark.parametrize('values,expected', [
    ([0.01, 1.0], True), ([0, 1], False), ([-1, 1], False),
    ([math.nan, 1], False), ([math.inf, 1], False), ([1e6, 1], False)])
def test_variance(values, expected):
    assert variance_ok(values, 1.5) == expected


def test_valid_route():
    assert validate_route({'waypoints': [{'latitude': 55, 'longitude': 37}]}) == [(55., 37., 0.)]


@pytest.mark.parametrize('data', [None, [], {}, {'waypoints': []}, {'waypoints': [None]},
    {'waypoints': [{'latitude': True, 'longitude': 10}]},
    {'waypoints': [{'latitude': '55', 'longitude': 10}]},
    {'waypoints': [{'latitude': math.nan, 'longitude': 10}]},
    {'waypoints': [{'latitude': 91, 'longitude': 10}]},
    {'waypoints': [{'latitude': 55, 'longitude': -181}]},
    {'waypoints': [{'latitude': 55}]},
    {'waypoints': [{'latitude': 55, 'longitude': 10, 'altitude': math.inf}]}])
def test_bad_route(data):
    with pytest.raises(ValueError):
        validate_route(data)


def test_legs():
    check_legs((0, 0), [(3, 4), (10, 10)], 15)
    with pytest.raises(ValueError):
        check_legs((0, 0), [(16, 0)], 15)
    with pytest.raises(ValueError):
        check_legs((0, 0), [(1, 0), (17, 0)], 15)
    with pytest.raises(ValueError):
        check_legs((0, 0), [(math.nan, 0)], 15)


def test_localization_contract():
    params = yaml.safe_load((ROOT / 'config/localization.yaml').read_text())
    assert 'ekf_local' not in params  # no second odom->base_link publisher
    launch = (ROOT / 'launch/localization.launch.py').read_text()
    assert "name='ekf_local'" not in launch
    assert "executable='counter_odometry'" in launch
    assert (ROOT.parent / 'tracked_robot_interfaces/msg/TrackTicks.msg').exists()
    local = params['counter_odometry']['ros__parameters']
    glob = params['ekf_global']['ros__parameters']
    assert local['odom_frame'] == 'odom'
    assert local['input_timeout'] <= 0.4
    assert glob['world_frame'] == 'map'
    assert glob['odom0'] == '/odometry/local'
    assert len(glob['odom0_config']) == len(glob['imu0_config']) == 15
    assert [i for i, v in enumerate(glob['odom0_config']) if v] == [6]
    assert [i for i, v in enumerate(glob['imu0_config']) if v] == [5, 11]
    assert glob['imu0_relative'] is False
    assert [i for i, v in enumerate(glob['odom1_config']) if v] == [0, 1]


def test_nav_config():
    params = yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())
    for name in ('global_costmap', 'local_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert costmap['rolling_window'] is True
        assert 'static_layer' not in costmap['plugins']
        scan = costmap['obstacle_layer']['scan']
        assert scan['topic'] == '/scan' and scan['marking'] and scan['clearing']
        assert scan['raytrace_max_range'] > scan['obstacle_max_range']
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert 'ObstacleFootprint' in controller['critics']
    monitor = params['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel/checked'
    assert monitor['StopZone']['action_type'] == 'stop'
    assert monitor['source_timeout'] <= 0.5
    assert params['velocity_smoother']['ros__parameters']['velocity_timeout'] <= 0.5


def test_example_is_disabled():
    data = yaml.safe_load((ROOT / 'config/route.example.yaml').read_text())
    assert data['enabled'] is False
    validate_route(data)


def test_behavior_tree_contract():
    tree = ET.parse(ROOT / 'behavior_trees/gps_navigation.xml')
    assert tree.getroot().attrib['BTCPP_format'] == '4'
    assert len(tree.findall('.//ComputePathToPose')) == 1
    assert len(tree.findall('.//FollowPath')) == 1
    assert not tree.findall('.//BackUp')
