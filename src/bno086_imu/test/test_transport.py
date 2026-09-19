from collections import deque
import pytest
from bno086_imu.protocol import ProtocolError, packet
from bno086_imu.transport import ShtpI2C


def transport(reads):
    driver = object.__new__(ShtpI2C)
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
    incoming = deque([(1, 0, b'\x01'), None, (2, 0, b'\xf8'+bytes(15))])
    driver.receive = lambda: incoming.popleft()
    writes = []
    driver.send = lambda channel, payload: writes.append((channel, payload))
    driver.configure(25, 10)
    assert writes[:2] == [(1, b'\x01'), (2, b'\xf9\x00')]
    assert [p[1] for _, p in writes[2:]] == [1, 2, 5, 3]
    assert all(channel == 2 and p[0] == 0xFD for channel, p in writes[2:])
