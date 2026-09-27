"""Small, strict SH-2/SHTP subset (not a general BNO08x SDK).

Wire definitions: CEVA SH-2 Reference Manual, sh2_SensorValue.c; see README.
Only calibrated acceleration/gyro/mag and magnetic Rotation Vector are enabled.
Unknown/truncated sensor reports and fragmented SHTP cargos fail closed.
"""
from dataclasses import dataclass
import math
import struct

ACCEL, GYRO, MAG, ROTATION = 0x01, 0x02, 0x03, 0x05
SIZES = {ACCEL: 10, GYRO: 10, MAG: 10, ROTATION: 14}
SCALES = {ACCEL: 256.0, GYRO: 512.0, MAG: 16_000_000.0, ROTATION: 16384.0}


class ProtocolError(ValueError):
    """Untrustworthy or unsupported wire data: do not publish it."""


@dataclass(frozen=True)
class Report:
    sensor: int
    sequence: int
    accuracy: int
    delay: float
    values: tuple
    heading_accuracy: float = 0.0


def header(data):
    if len(data) != 4:
        raise ProtocolError('SHTP header must be 4 bytes')
    raw_length, channel, sequence = struct.unpack('<HBB', data)
    length = raw_length & 0x7FFF
    if raw_length == 0xFFFF or channel > 5 or (length and not 4 <= length <= 4096):
        raise ProtocolError('Invalid SHTP header')
    return length, channel, sequence, bool(raw_length & 0x8000)


def packet(channel, sequence, payload):
    if not 0 <= channel <= 5 or not 0 <= sequence <= 255 or len(payload) > 4092:
        raise ProtocolError('Invalid outgoing SHTP packet')
    return struct.pack('<HBB', len(payload) + 4, channel, sequence) + payload


def set_feature(sensor, rate_hz):
    if sensor not in SIZES or not math.isfinite(rate_hz) or not 1 <= rate_hz <= 100:
        raise ValueError('Supported report rate is 1..100 Hz')
    # FD, sensor, flags, change sensitivity, interval us, batch us, config.
    return struct.pack('<BBBHIII', 0xFD, sensor, 0, 0, round(1e6 / rate_hz), 0, 0)


def decode_reports(payload):
    """Decode a complete input cargo, atomically; mag is returned in tesla."""
    reports = []
    offset = 0
    while offset < len(payload):
        sensor = payload[offset]
        if sensor in (0xFB, 0xFA):  # base timestamp / timestamp rebase
            if len(payload) - offset < 5:
                raise ProtocolError('Truncated timestamp report')
            offset += 5
            continue
        length = SIZES.get(sensor)
        if length is None:
            raise ProtocolError(f'Unsupported sensor report 0x{sensor:02x}')
        if len(payload) - offset < length:
            raise ProtocolError('Truncated sensor report')
        raw = payload[offset:offset + length]
        count = 4 if sensor == ROTATION else 3
        values = tuple(v / SCALES[sensor] for v in struct.unpack_from('<' + 'h'*count, raw, 4))
        # SH-2 delay: status bits 7:2 are delay[13:8], byte 3 is delay[7:0].
        delay = (((raw[2] & 0xFC) << 6) | raw[3]) * 0.0001
        accuracy = struct.unpack_from('<h', raw, 12)[0] / 4096.0 if sensor == ROTATION else 0.0
        if accuracy < 0:
            raise ProtocolError('Negative Rotation Vector heading error')
        reports.append(Report(sensor, raw[1], raw[2] & 3, delay, values, accuracy))
        offset += length
    return reports


def true_enu_quaternion(q, declination_deg):
    """Magnetic ENU -> true ENU. East-positive declination is SUBTRACTED.

    q is (x,y,z,w), sensor -> magnetic world, as in SH-2 Rotation Vector.
    Physical sensor mounting is exclusively in base_link -> imu_link TF.
    """
    if len(q) != 4 or not all(math.isfinite(v) for v in (*q, declination_deg)):
        raise ValueError('Nonfinite quaternion/declination')
    norm = math.sqrt(sum(v*v for v in q))
    if not 0.9 <= norm <= 1.1:
        raise ValueError('Invalid quaternion norm')
    x, y, z, w = (v / norm for v in q)
    angle = -math.radians(declination_deg) / 2.0
    s, c = math.sin(angle), math.cos(angle)
    return (c*x - s*y, c*y + s*x, c*z + s*w, c*w - s*z)


def decode_product_id(payload):
    """SH-2 Product ID shared by BNO085/BNO086; not a silicon model detector."""
    if len(payload) != 16 or payload[0] != 0xF8:
        raise ProtocolError('Expected a 16-byte SH-2 Product ID response (0xF8)')
    _, cause, major, minor, part, build, patch, _ = struct.unpack('<BBBBIIHH', payload)
    return dict(reset_cause=cause, version=f'{major}.{minor}.{patch}',
                software_part=part, software_build=build)


def complete_i2c_packet(first, data):
    """Validate a header-only probe followed by the remaining full I2C cargo.

    BNO085 can consume the probe as a header-only transfer: the next header has
    continuation=1 and sequence+1, with unchanged cargo length and channel. No
    payload was consumed by the 4-byte probe. Also accept exact header replay.
    This is NOT arbitrary multi-fragment payload reassembly.
    """
    length, channel, sequence, continuation = header(first)
    if not length or continuation:
        raise ProtocolError('Unexpected empty/orphan continuation probe')
    if len(data) != length:
        raise ProtocolError(f'Short I2C read: expected {length}, got {len(data)}')
    second_length, second_channel, second_seq, second_cont = header(data[:4])
    if (second_length, second_channel) != (length, channel):
        raise ProtocolError('SHTP length/channel changed after header probe')
    replay = data[:4] == first
    header_only_continuation = second_cont and second_seq == (sequence + 1) % 256
    if not (replay or header_only_continuation):
        raise ProtocolError('SHTP sequence/continuation mismatch after header probe')
    return second_channel, second_seq, data[4:]


def decode_control_products(payload):
    """Parse bundled SH-2 control reports at report boundaries, never scan for F8.

    Supported incoming reports: F1 command response (16), F8 Product ID (16),
    FC Get Feature response (17). Fail closed on unknown/truncated reports.
    """
    sizes = {0xF1: 16, 0xF8: 16, 0xFC: 17}
    products = []
    offset = 0
    while offset < len(payload):
        report_id = payload[offset]
        size = sizes.get(report_id)
        if size is None or offset + size > len(payload):
            raise ProtocolError(f'Unknown/truncated SH-2 control report 0x{report_id:02x} at {offset}')
        if report_id == 0xF8:
            products.append(decode_product_id(payload[offset:offset + size]))
        offset += size
    return products
