"""Static wiring contract to the existing robot_localization/Nav2 setup."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[3]


def test_old_package_removed_and_new_dependency_declared():
    assert not (ROOT / 'src/imu_stm32_bridge').exists()
    manifest = ET.parse(ROOT / 'src/project_start/package.xml')
    deps = [node.text for node in manifest.findall('.//exec_depend')]
    assert 'bno086_imu' in deps
    assert 'imu_stm32_bridge' not in deps


def test_imu_defaults_and_launch_contract():
    package = ROOT / 'src/bno086_imu'
    config = yaml.safe_load((package / 'config/imu.yaml').read_text())['/**/bno086_imu']['ros__parameters']
    assert config['i2c_address'] == 0x4B
    assert config['frame_id'] == 'imu_link'
    assert config['min_accuracy'] >= 2
    assert config['sample_max_age'] <= .2
    assert (package / 'resource/bno086_imu').exists()
    assert (package / 'README.md').exists()
    start = (ROOT / 'src/project_start/launch/start.launch.py').read_text()
    assert "package='bno086_imu'" in start
    assert 'imu_port' not in start
    assert 'imu_i2c_bus' in start and 'imu_i2c_address' in start


def test_no_double_declination_and_no_acceleration_fusion():
    config = yaml.safe_load((ROOT / 'src/project_start/config/localization.yaml').read_text())
    assert config['navsat_transform']['ros__parameters']['magnetic_declination_radians'] == 0.0
    assert config['navsat_transform']['ros__parameters']['yaw_offset'] == 0.0
    for name in ('ekf_global',):
        p = config[name]['ros__parameters']
        assert p['imu0'] == '/imu/data'
        assert [i for i, enabled in enumerate(p['imu0_config']) if enabled] == [5, 11]


def test_sensor_frame_exists_in_urdf():
    tree = ET.parse(ROOT / 'src/tracked_robot_description/urdf/tracked_robot.urdf.xacro')
    assert tree.find(".//link[@name='imu_link']") is not None
    joint = tree.find(".//joint[@name='base_to_imu']")
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'imu_link'
