"""Regression against the user's BNO085 / 0x4B / 100 kHz capture."""
from collections import deque
from types import SimpleNamespace
import pytest
from bno08x_imu.protocol import (
    ProtocolError, complete_i2c_packet, decode_control_products, packet,
)
from bno08x_imu.transport import ShtpI2C
from bno08x_imu.diagnose import Diagnostic

# Complete 84-byte transfer copied from the user's log, not a generated packet.
CAPTURE = bytes.fromhex('''
54 80 02 01 f1 00 84 00 00 00 01 00 00 00 00 00 00 00 00 00
f8 04 03 02 b4 a6 98 00 06 00 00 00 0d 00 00 00
f8 00 01 02 96 a4 98 00 e6 00 00 00 04 00 00 00
f8 00 04 04 36 a3 98 00 e5 01 00 00 03 00 00 00
f8 00 04 02 e3 a2 98 00 24 02 00 00 0a 00 00 00
''')
FIRST = bytes.fromhex('54 00 02 00')


def test_captured_control_packet_and_all_product_ids():
    channel, seq, payload = complete_i2c_packet(FIRST, CAPTURE)
    assert len(CAPTURE) == 84
    assert (channel, seq) == (2, 1)
    products = decode_control_products(payload)
    assert [p['version'] for p in products] == ['3.2.13', '1.2.4', '4.4.3', '4.2.10']
    assert [p['software_build'] for p in products] == [6, 230, 485, 548]
    assert products[0]['software_part'] == 0x98A6B4
    assert products[0]['reset_cause'] == 4


def test_captured_reset_notification():
    assert complete_i2c_packet(bytes.fromhex('05 00 01 00'),
                               bytes.fromhex('05 80 01 01 01')) == (1, 1, b'\x01')


def test_advertisement_header_pair_with_synthetic_payload():
    # Only a prefix of the real 276-byte payload was printed; don't invent its contents.
    first = bytes.fromhex('14 01 00 00')
    body = bytes.fromhex('14 81 00 01') + bytes(272)
    assert complete_i2c_packet(first, body) == (0, 1, bytes(272))


def test_sequence_wrap():
    assert complete_i2c_packet(bytes.fromhex('05 00 01 ff'),
                               bytes.fromhex('05 80 01 00 01')) == (1, 0, b'\x01')


@pytest.mark.parametrize('first,body', [
    ('05 00 01 00', '05 80 01 02 01'),  # wrong sequence
    ('05 00 01 00', '05 80 02 01 01'),  # wrong channel
    ('05 00 01 00', '06 80 01 01 01'),  # wrong length
    ('05 00 01 00', '05 80 01 01'),     # short transfer
    ('05 80 01 00', '05 80 01 01 01'),  # orphan continuation
    ('05 00 01 00', '05 00 01 01 01'),  # sequence changed without continuation
])
def test_not_a_blanket_header_check_bypass(first, body):
    with pytest.raises(ProtocolError):
        complete_i2c_packet(bytes.fromhex(first), bytes.fromhex(body))


def test_replayed_header_still_supported():
    data = packet(3, 17, b'abc')
    assert complete_i2c_packet(data[:4], data) == (3, 17, b'abc')


def test_control_report_boundaries_not_byte_search():
    assert decode_control_products(b'\xf1' + b'\xf8' * 15) == []
    with pytest.raises(ProtocolError):
        decode_control_products(b'\x99' + CAPTURE[20:])
    with pytest.raises(ProtocolError):
        decode_control_products(CAPTURE[4:-1])


def fake_transport(reads):
    t = object.__new__(ShtpI2C)
    q = deque(reads)
    def read(size):
        value = q.popleft()
        assert len(value) == size
        return value
    t.read_bytes = read
    t.gpio = SimpleNamespace(ready=lambda: True, reset=lambda: None,
                             wait_ready=lambda _: None)
    t.send = lambda *args: None
    return t


def test_production_startup_accepts_real_capture(monkeypatch):
    monkeypatch.setattr('bno08x_imu.transport.time.sleep', lambda _: None)
    t = fake_transport([bytes(4), FIRST, CAPTURE])
    product = t.configure(25, 10)
    assert product['version'] == '3.2.13'
    assert len(product['components']) == 4


def test_diagnostic_accepts_real_capture():
    t = fake_transport([bytes(4), FIRST, CAPTURE])
    now = [0.0]
    d = Diagnostic(t, log=lambda _: None, clock=lambda: now[0],
                   sleep=lambda dt: now.__setitem__(0, now[0] + dt))
    d.reset_and_observe = lambda: None
    result = d.attempt(0x4B)
    assert result['product_id']['version'] == '3.2.13'
    assert len(result['product_ids']) == 4
    assert result['errors'] == []
