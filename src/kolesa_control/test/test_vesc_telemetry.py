import struct
import pytest
from kolesa_control import vesc_protocol as vp


def telemetry(signed=-1234, absolute=9876, fault=0):
    payload = bytearray([vp.Comm.GET_VALUES])
    for name, kind, scale, *_ in vp._VALUES_FIELDS:
        value = signed if name == 'tachometer' else absolute if name == 'tachometer_abs' else 0
        payload += struct.pack('>h' if kind == 'i16' else '>i', value)
    payload.append(fault)
    return bytes(payload)


def test_signed_counters_crc_and_trailing_extension():
    payload = telemetry()
    data = vp.parse_get_values(payload + b'\x01\x02\x03')
    assert data['tachometer'] == -1234
    assert data['tachometer_abs'] == 9876
    assert data['fault_code'] == 0
    decoder = vp.PacketDecoder()
    packet = vp.encode_packet(payload)
    assert decoder.feed(packet[:10]) == []
    assert decoder.feed(packet[10:]) == [payload]


def test_every_truncated_prefix_is_rejected():
    payload = telemetry()
    for length in range(len(payload)):
        assert vp.parse_get_values(payload[:length]) is None


def test_corrupted_crc_never_updates_counter():
    frame = bytearray(vp.encode_packet(telemetry()))
    frame[-4] ^= 0x01
    assert vp.PacketDecoder().feed(frame) == []


@pytest.mark.parametrize('value', [-2147483648, 2147483647, 0])
def test_int32_limits(value):
    data = vp.parse_get_values(telemetry(value, value))
    assert data['tachometer'] == data['tachometer_abs'] == value
