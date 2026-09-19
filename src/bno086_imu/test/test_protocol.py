"""Synthetic SH-2 fixtures, independent of ROS and physical I2C."""
import math
import struct
import pytest
from bno086_imu.protocol import (
    ACCEL, GYRO, MAG, ROTATION, ProtocolError, decode_reports,
    header, packet, set_feature, true_enu_quaternion,
)


def vector(sensor, values, seq=1, accuracy=3):
    return bytes([sensor, seq, accuracy, 0]) + struct.pack('<hhh', *values)


def rotation(seq=1, accuracy=3):
    return bytes([ROTATION, seq, accuracy, 0]) + struct.pack('<hhhhh', 0, 0, 0, 16384, 512)


def test_units_multiple_reports_and_timestamps():
    raw = (b'\xfb' + struct.pack('<I', 1234)
           + vector(ACCEL, (256, -512, 2511))
           + vector(GYRO, (512, -256, 0))
           + b'\xfa' + struct.pack('<i', -100)
           + vector(MAG, (160, -320, 640)) + rotation())
    reports = decode_reports(raw)
    assert reports[0].values == (1., -2., 2511/256)
    assert reports[1].values == (1., -0.5, 0.)
    assert reports[2].values == pytest.approx((10e-6, -20e-6, 40e-6))
    assert reports[3].values == (0., 0., 0., 1.)
    assert reports[3].heading_accuracy == 0.125  # Q12, not Q14
    assert reports[3].accuracy == 3


def test_delay_and_status_bits():
    raw = bytearray(rotation())
    raw[2] = 0x06  # medium accuracy + upper delay bit
    raw[3] = 0x10
    report = decode_reports(raw)[0]
    assert report.delay == pytest.approx(272 * 0.0001)
    assert report.accuracy == 2


@pytest.mark.parametrize('payload', [b'\xfb', b'\xfa\x00', b'\x05', rotation()[:-1],
                                     b'\x08' + bytes(11), b'\x99', vector(GYRO, (0,0,0)) + b'\xff'])
def test_bad_reports_are_atomic(payload):
    with pytest.raises(ProtocolError):
        decode_reports(payload)


def test_negative_heading_error_rejected():
    raw = rotation()[:-2] + struct.pack('<h', -1)
    with pytest.raises(ProtocolError):
        decode_reports(raw)


def test_feature_configuration():
    data = set_feature(ROTATION, 25.0)
    assert len(data) == 17
    assert struct.unpack('<BBBHIII', data) == (0xFD, 5, 0, 0, 40000, 0, 0)
    assert set_feature(MAG, 10.0)[5:9] == struct.pack('<I', 100000)


@pytest.mark.parametrize('rate', [0, -1, 101, math.inf, math.nan])
def test_bad_rate(rate):
    with pytest.raises(ValueError):
        set_feature(ROTATION, rate)


def test_game_vector_is_not_supported():
    with pytest.raises(ValueError):
        set_feature(8, 25)


def test_packet_header():
    raw = packet(2, 255, set_feature(ROTATION, 25))
    assert header(raw[:4]) == (21, 2, 255, False)
    assert header(bytes(4)) == (0, 0, 0, False)
    assert header(b'\x10\x80\x03\x01') == (16, 3, 1, True)


@pytest.mark.parametrize('data', [b'', b'\x01\x00\x03\x00', b'\xff'*4,
                                  b'\x04\x00\x06\x00', b'\x01\x10\x03\x00'])
def test_bad_header(data):
    with pytest.raises(ProtocolError):
        header(data)


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


@pytest.mark.parametrize('angle', [0, 90, 180, -90])
def test_enu_cardinal_directions(angle):
    a = math.radians(angle) / 2
    q = true_enu_quaternion((0, 0, math.sin(a), math.cos(a)), 0)
    assert yaw(q) == pytest.approx(math.radians(angle))


def test_east_positive_declination_sign():
    # X along magnetic north with +10 degree east declination points at
    # true ENU yaw +80 degrees, not +100 degrees.
    q = true_enu_quaternion((0, 0, math.sqrt(0.5), math.sqrt(0.5)), 10)
    assert yaw(q) == pytest.approx(math.radians(80))
    assert sum(v*v for v in q) == pytest.approx(1)


def test_declination_preserves_body_tilt():
    # Rz(-D) * Rx(90), not Rx(90) * Rz(-D): world correction, not mount.
    q = true_enu_quaternion((math.sqrt(0.5), 0, 0, math.sqrt(0.5)), 90)
    assert q == pytest.approx((0.5, -0.5, -0.5, 0.5))


@pytest.mark.parametrize('q', [(0,0,0,0), (0,0,0,2), (0,math.nan,0,1), (0,0,math.inf,1), (0,0,1)])
def test_invalid_quaternion(q):
    with pytest.raises(ValueError):
        true_enu_quaternion(q, 0)
