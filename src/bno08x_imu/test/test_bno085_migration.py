"""BNO085 migration contract and shared SH-2 Product ID validation."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import pytest
import yaml
from bno08x_imu.protocol import decode_product_id, ProtocolError

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / 'src/bno08x_imu'


def test_package_and_entrypoints_consistent():
    assert not (ROOT / 'src/bno086_imu').exists()
    assert ET.parse(PKG / 'package.xml').findtext('name') == 'bno08x_imu'
    assert (PKG / 'resource/bno08x_imu').exists()
    assert (PKG / 'bno08x_imu/__init__.py').exists()
    setup = (PKG / 'setup.py').read_text()
    assert 'imu_node = bno08x_imu.node:main' in setup
    assert 'bno08x_diagnose = bno08x_imu.diagnose:main' in setup
    assert 'bno086_imu' not in setup
    assert '$base/lib/bno08x_imu' in (PKG / 'setup.cfg').read_text()
    assert "super().__init__('bno08x_imu')" in (PKG / 'bno08x_imu/node.py').read_text()


def test_wiring_and_clock_contract():
    p = yaml.safe_load((PKG / 'config/imu.yaml').read_text())['/**/bno08x_imu']['ros__parameters']
    assert (p['i2c_bus'], p['rst_gpio'], p['int_gpio']) == (1, 17, 27)
    clock = (PKG / 'config/pi5_i2c_100khz.txt').read_text()
    assert 'dtparam=i2c_arm=on' in clock
    assert 'dtparam=i2c_arm_baudrate=100000' in clock
    assert 'rate_hz' not in p or p['rate_hz'] == 25.
    assert 'config/*.txt' in (PKG / 'setup.py').read_text()


@pytest.mark.parametrize('part,build,major,minor,patch', [
    (123, 456, 3, 2, 7), (0x12345678, 0xABCDEF01, 1, 9, 258)])
def test_product_id_is_protocol_not_model_whitelist(part, build, major, minor, patch):
    # Synthetic SH-2 bytes; do not pretend these are measured chip identities.
    raw = struct.pack('<BBBBIIHH', 0xF8, 1, major, minor, part, build, patch, 0)
    assert decode_product_id(raw) == dict(reset_cause=1,
        version=f'{major}.{minor}.{patch}', software_part=part, software_build=build)


@pytest.mark.parametrize('raw', [b'', b'\xf8', b'\xf8' + bytes(14),
                                 b'\xf8' + bytes(16), b'\xf9' + bytes(15)])
def test_bad_product_id_rejected(raw):
    with pytest.raises(ProtocolError):
        decode_product_id(raw)
