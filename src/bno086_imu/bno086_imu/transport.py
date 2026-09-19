"""Raw I2C_RDWR SHTP transport: no SMBus register/block-length protocol."""
import time
import fcntl
from .protocol import ACCEL, GYRO, MAG, ROTATION, ProtocolError, header, packet, set_feature


class ShtpI2C:
    def __init__(self, bus_number, address):
        # Lazy import lets protocol and fake-bus tests run without hardware packages.
        from smbus2 import SMBus, i2c_msg
        self.bus = SMBus(bus_number)
        try:
            # Cooperative exclusive bus ownership: a second copy must not reset
            # this sensor underneath the running navigation stack.
            fcntl.flock(self.bus.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            self.bus.close()
            raise
        self.message = i2c_msg
        self.address = address
        self.tx_sequence = [0] * 6

    def close(self):
        self.bus.close()

    def read_bytes(self, size):
        msg = self.message.read(self.address, size)
        self.bus.i2c_rdwr(msg)
        return bytes(msg)

    def send(self, channel, payload):
        data = packet(channel, self.tx_sequence[channel], payload)
        self.bus.i2c_rdwr(self.message.write(self.address, data))
        self.tx_sequence[channel] = (self.tx_sequence[channel] + 1) & 255

    def receive(self):
        first = self.read_bytes(4)
        length, channel, sequence, continuation = header(first)
        if length == 0:
            return None
        # Every I2C read starts with an SHTP header again. Read entire packet
        # in one Linux I2C message; SMBus read_i2c_block_data's 32-byte limit is wrong here.
        data = self.read_bytes(length)
        if len(data) != length or data[:4] != first:
            raise ProtocolError('SHTP header changed or short I2C read')
        if continuation:
            raise ProtocolError('Fragmented SHTP cargo is not supported')
        return channel, sequence, data[4:]

    def configure(self, rate_hz, mag_rate_hz):
        self.send(1, b'\x01')  # executable channel: soft reset
        time.sleep(0.3)
        # Drain boot advertisements/reset notifications, never expose them as measurements.
        deadline = time.monotonic() + 2.0
        while self.receive() is not None:
            if time.monotonic() >= deadline:
                raise TimeoutError('BNO086 boot stream did not settle')
        self.send(2, b'\xf9\x00')  # Product ID request confirms SH-2 is running.
        deadline = time.monotonic() + 2.0
        while True:
            item = self.receive()
            if item and item[0] == 2 and len(item[2]) >= 16 and item[2][0] == 0xF8:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('No SH-2 Product ID response')
            time.sleep(0.01)
        for sensor in (ACCEL, GYRO, ROTATION, MAG):
            self.send(2, set_feature(sensor, mag_rate_hz if sensor == MAG else rate_hz))
            time.sleep(0.01)
