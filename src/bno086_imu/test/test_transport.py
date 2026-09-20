from collections import deque
from types import SimpleNamespace
import pytest
from bno086_imu.protocol import ProtocolError, packet
from bno086_imu.transport import ShtpI2C


def transport(reads):
    driver = object.__new__(ShtpI2C)
    driver.gpio = SimpleNamespace(ready=lambda: True)
    queue = deque(reads)
    def read(size):
        result = queue.popleft()
        assert len(result) == size
        return result
    driver.read_bytes = read
    return driver


def test_full_read_repeated_header():
    data = packet(3, 42, bytes(50))
    driver = transport([data[:4], data])
    assert driver.receive() == (3, 42, bytes(50))


def test_empty():
    assert transport([bytes(4)]).receive() is None


def test_header_changed():
    a, b = packet(3,1,b'abc'), packet(3,2,b'abc')
    with pytest.raises(ProtocolError):
        transport([a[:4], b]).receive()


def test_fragment_rejected():
    data = b'\x07\x80\x03\x01abc'
    with pytest.raises(ProtocolError):
        transport([data[:4], data]).receive()


def test_write_counter_wrap_and_channels():
    class Message:
        @staticmethod
        def write(address, data):
            assert address == 75
            return data
    class Bus:
        def __init__(self):
            self.writes = []
        def i2c_rdwr(self, data):
            self.writes.append(data)
    driver = object.__new__(ShtpI2C)
    driver.bus, driver.message, driver.address = Bus(), Message, 75
    driver.tx_sequence = [0]*6
    driver.tx_sequence[2] = 255
    driver.send(2, b'abc')
    driver.send(2, b'def')
    driver.send(1, b'\x01')
    assert [data[3] for data in driver.bus.writes] == [255, 0, 0]
    assert driver.bus.writes[-1] == b'\x05\x00\x01\x00\x01'


def test_read_uses_raw_i2c_rdwr():
    class Message:
        @staticmethod
        def read(address, size):
            assert address == 75
            return bytearray(size)
    class Bus:
        def i2c_rdwr(self, msg):
            msg[:] = b'\x04\x00\x03\x01'
    driver = object.__new__(ShtpI2C)
    driver.bus, driver.message, driver.address = Bus(), Message, 75
    assert driver.read_bytes(4) == b'\x04\x00\x03\x01'


def test_configure_reset_identification_and_four_reports(monkeypatch):
    monkeypatch.setattr('bno086_imu.transport.time.sleep', lambda _: None)
    driver = object.__new__(ShtpI2C)
    actions = []
    driver.gpio = SimpleNamespace(reset=lambda: actions.append("reset"),
                                  wait_ready=lambda timeout: actions.append(timeout))
    incoming = deque([(1, 0, b'\x01'), None, (2, 0, b'\xf8'+bytes(15))])
    driver.receive = lambda: incoming.popleft()
    writes = []
    driver.send = lambda channel, payload: writes.append((channel, payload))
    driver.configure(25, 10)
    assert actions == ['reset', 2.0]
    assert writes[:1] == [(2, b'\xf9\x00')]
    assert [p[1] for _, p in writes[1:]] == [1, 2, 5, 3]
    assert all(channel == 2 and p[0] == 0xFD for channel, p in writes[1:])


def test_int_high_means_no_bus_read():
    driver = transport([])
    driver.gpio.ready = lambda: False
    assert driver.receive() is None


def test_int_checked_once_for_whole_packet():
    data = packet(3, 42, bytes(50))
    driver = transport([data[:4], data])
    levels = iter([True, False])
    driver.gpio.ready = lambda: next(levels)
    assert driver.receive() == (3, 42, bytes(50))
    assert driver.receive() is None


def test_failed_gpio_acquisition_closes_bus(monkeypatch):
    import sys
    calls = []
    bus = SimpleNamespace(fd=123, close=lambda: calls.append('closed'))
    monkeypatch.setitem(sys.modules, 'smbus2', SimpleNamespace(SMBus=lambda _: bus, i2c_msg=None))
    monkeypatch.setattr('bno086_imu.transport.fcntl.flock', lambda *_: calls.append('lock'))
    def unavailable(*args):
        calls.append('gpio')
        raise PermissionError('GPIO access denied')
    monkeypatch.setattr('bno086_imu.gpio.SensorGPIO', unavailable)
    with pytest.raises(PermissionError):
        ShtpI2C(1, 75)
    assert calls == ['lock', 'gpio', 'closed']


def test_bus_lock_failure_never_touches_gpio(monkeypatch):
    import sys
    calls = []
    bus = SimpleNamespace(fd=123, close=lambda: calls.append('closed'))
    monkeypatch.setitem(sys.modules, 'smbus2', SimpleNamespace(SMBus=lambda _: bus, i2c_msg=None))
    def busy(*args): raise BlockingIOError('busy bus')
    monkeypatch.setattr('bno086_imu.transport.fcntl.flock', busy)
    monkeypatch.setattr('bno086_imu.gpio.SensorGPIO', lambda *_: calls.append('unexpected GPIO'))
    with pytest.raises(BlockingIOError):
        ShtpI2C(1, 75)
    assert calls == ['closed']


def test_boot_timeout_does_not_send_reports():
    driver = object.__new__(ShtpI2C)
    writes = []
    def timeout(_): raise TimeoutError('INT')
    driver.gpio = SimpleNamespace(reset=lambda: None, wait_ready=timeout)
    driver.send = lambda *args: writes.append(args)
    with pytest.raises(TimeoutError):
        driver.configure(25, 10)
    assert writes == []


def test_product_id_timeout(monkeypatch):
    driver = object.__new__(ShtpI2C)
    driver.gpio = SimpleNamespace(reset=lambda: None, wait_ready=lambda _: None)
    driver.receive = lambda: None
    writes = []
    driver.send = lambda *args: writes.append(args)
    clock = iter([0., 0., 3.])
    monkeypatch.setattr('bno086_imu.transport.time.monotonic', lambda: next(clock))
    with pytest.raises(TimeoutError, match='Product ID'):
        driver.configure(25, 10)
    assert writes == [(2, b'\xf9\x00')]
