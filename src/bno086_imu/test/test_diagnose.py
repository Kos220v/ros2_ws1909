"""Bench-only diagnostic tests: no ROS, GPIO or real I2C required."""
from collections import deque
from types import SimpleNamespace
import struct
import pytest
from bno086_imu import diagnose
from bno086_imu.protocol import packet, ProtocolError


class Clock:
    def __init__(self):
        self.now = 0.0
    def __call__(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


class Transport:
    def __init__(self, clock):
        self.clock = clock
        self.address = 0x4B
        self.tx_sequence = [0] * 6
        self.reset_events = []
        self.samples = []
        self.writes = []
        self.reads = []
        self.incoming = deque()
        self.closed = False
        self.gpio = SimpleNamespace(ready=self.ready, _reset_level=self.level,
                                    path='/dev/gpiochip4')
    def ready(self):
        self.samples.append(self.clock())
        return False
    def level(self, high):
        self.reset_events.append((self.clock(), high))
        if not high:
            self.incoming.clear()
    def read_bytes(self, length):
        self.reads.append((self.clock(), self.address, length))
        if not self.incoming:
            return bytes(length)
        data = self.incoming[0]
        if length == 4:
            return data[:4]
        self.incoming.popleft()
        assert len(data) == length
        return data
    def send(self, channel, payload):
        self.writes.append((self.address, channel, payload))
        if self.address == 0x4B:
            reply = struct.pack('<BBBBIIHH', 0xF8, 1, 3, 2, 123, 456, 7, 0)
            self.incoming.append(packet(2, 0, reply))
    def close(self):
        self.closed = True


def make():
    clock = Clock()
    t = Transport(clock)
    logs = []
    d = diagnose.Diagnostic(t, logs.append, clock, clock.sleep)
    return d, t, logs


def test_product_id_even_when_int_always_high():
    d, t, logs = make()
    result = d.attempt(0x4B)
    assert not result['int_low']
    assert result['product_id'] == dict(reset_cause=1, version='3.2.7',
                                      software_part=123, software_build=456)
    assert t.writes == [(0x4B, 2, b'\xf9\x00')]
    assert t.reset_events[0][1] is False
    assert t.reset_events[1][1] is True
    release = t.reset_events[1][0]
    assert any(release <= sample <= release + .002 for sample in t.samples)
    assert t.reads[0][0] >= release + 2.
    assert any('VALID SH-2 Product ID' in line for line in logs)


def test_separate_reset_for_each_address_and_no_reports_enabled():
    d, t, _ = make()
    assert d.attempt(0x4A)['product_id'] is None
    assert d.attempt(0x4B)['product_id'] is not None
    assert [v for _, v in t.reset_events] == [False, True, False, True]
    assert t.writes == [(0x4A, 2, b'\xf9\x00'), (0x4B, 2, b'\xf9\x00')]


def test_poll_observes_early_low_without_300ms_blind_sleep():
    d, t, logs = make()
    t.gpio.ready = lambda: .020 <= d.clock() <= .040
    d.reset_and_observe()
    assert d.seen_low
    assert any('INT=0' in s for s in logs)


def test_errno_reported_and_no_repeated_writes():
    d, t, logs = make()
    def fail(*_):
        raise OSError(121, 'Remote I/O error')
    t.read_bytes = t.send = fail
    result = d.attempt(0x4B)
    assert result['product_id'] is None
    assert len(result['errors']) == 2
    assert any('errno=121' in s for s in logs)


def test_invalid_header_does_not_allocate_large_read():
    d, t, _ = make()
    t.incoming.append(b'\xff\xff\xff\xff')
    with pytest.raises(ProtocolError):
        d.raw_receive()
    assert len(t.reads) == 1


def test_repeated_header_mismatch():
    d, t, _ = make()
    a, b = packet(2, 0, b'abc'), packet(2, 1, b'abc')
    replies = iter([a[:4], b])
    t.read_bytes = lambda _: next(replies)
    with pytest.raises(ProtocolError, match='mismatch'):
        d.raw_receive()


def test_malformed_product_id_not_success():
    d, t, _ = make()
    t.send = lambda *_: t.incoming.append(packet(2, 0, b'\xf8\x01'))
    assert d.attempt(0x4B)['product_id'] is None


def test_reset_released_if_observation_interrupted():
    d, t, _ = make()
    def interrupted(_):
        raise KeyboardInterrupt()
    d.observe = interrupted
    with pytest.raises(KeyboardInterrupt):
        d.reset_and_observe()
    assert [v for _, v in t.reset_events] == [False, True]


@pytest.mark.parametrize('args', [[], ['--confirm-stationary', '--rst-gpio', '2'],
    ['--confirm-stationary', '--int-gpio', '17'],
    ['--confirm-stationary', '--addresses', '0x50'],
    ['--confirm-stationary', '--i2c-bus', '-1']])
def test_arguments_rejected_before_hardware(args, monkeypatch):
    def unexpected(*_):
        pytest.fail('must not open hardware')
    monkeypatch.setattr(diagnose, 'ShtpI2C', unexpected)
    with pytest.raises(SystemExit) as error:
        diagnose.main(args)
    assert error.value.code == 2


@pytest.mark.parametrize('interrupt', [False, True])
def test_main_releases_resources_on_failure(monkeypatch, interrupt):
    d, t, _ = make()
    monkeypatch.setattr(diagnose, 'ShtpI2C', lambda *_: t)
    def fail(*_):
        if interrupt:
            raise KeyboardInterrupt()
        raise OSError('GPIO failed')
    monkeypatch.setattr(diagnose.Diagnostic, 'attempt', fail)
    assert diagnose.main(['--confirm-stationary']) == (130 if interrupt else 2)
    assert t.closed
    assert t.reset_events[-1][1] is True
