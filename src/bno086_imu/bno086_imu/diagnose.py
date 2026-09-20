"""Stationary bench diagnostic, independent of ROS. Never publishes measurements.

Deliberately probes raw SHTP after an INT timeout. This is NOT a fallback mode
for the production driver. GPIO polling is not an oscilloscope/edge recorder.
"""
import argparse
import json
import struct
import time

from .protocol import ProtocolError, header
from .transport import ShtpI2C


class Diagnostic:
    def __init__(self, transport, log=print, clock=time.monotonic, sleep=time.sleep):
        self.t = transport
        self.log = log
        self.clock, self.sleep = clock, sleep
        self.started = clock()
        self.previous_int = None
        self.seen_low = False

    def emit(self, text):
        self.log(f'[{self.clock() - self.started:8.3f}s] {text}')

    def sample_int(self):
        low = self.t.gpio.ready()
        level = 0 if low else 1
        if level != self.previous_int:
            self.emit(f'INT={level}')
            self.previous_int = level
        self.seen_low |= low
        return low

    def observe(self, seconds):
        deadline = self.clock() + seconds
        while True:
            self.sample_int()
            if self.clock() >= deadline:
                return
            self.sleep(0.001)

    def reset_and_observe(self):
        self.previous_int = None
        self.seen_low = False
        # The caller owns RST exclusively. Do not use production reset(), which
        # sleeps 300 ms after release. Observe throughout and immediately after.
        self.sample_int()
        self.emit('RST=0 (10 ms)')
        self.t.gpio._reset_level(False)
        try:
            self.observe(0.01)
        finally:
            self.t.gpio._reset_level(True)
        self.emit('RST=1; observing INT for 2 s, no I2C transactions yet')
        self.seen_low = False  # summary concerns readiness AFTER reset release
        self.observe(2.0)
        self.t.tx_sequence = [0] * 6
        self.emit(f'Post-reset INT LOW observed: {self.seen_low}')

    def raw_receive(self):
        # Intentionally not transport.receive(): diagnostic reads ignore INT.
        first = self.t.read_bytes(4)
        self.emit('RX header: ' + first.hex(' '))
        length, channel, seq, continuation = header(first)
        if length == 0:
            return None
        if continuation:
            raise ProtocolError('Fragmented SHTP cargo: diagnostic will not interpret it')
        # header() bounds allocation/transfer to 4096 bytes.
        data = self.t.read_bytes(length)
        self.emit(f'RX {len(data)} bytes: {data[:96].hex(" ")}' +
                  (' ... (hex truncated)' if len(data) > 96 else ''))
        if len(data) != length or data[:4] != first:
            raise ProtocolError('Short packet or repeated SHTP header mismatch')
        return channel, seq, data[4:]

    def attempt(self, address):
        self.t.address = address
        result = dict(address=f'0x{address:02x}', int_low=False,
                      product_id=None, errors=[])
        self.emit(f'=== Separate attempt at address 0x{address:02x} ===')
        self.reset_and_observe()
        result['int_low'] = self.seen_low
        self.emit('DIAGNOSTIC ONLY: raw I2C reads are allowed even with INT=HIGH')

        def read():
            self.sample_int()
            try:
                return self.raw_receive()
            except (OSError, ProtocolError) as error:
                detail = f'{type(error).__name__}: {error}'
                if isinstance(error, OSError):
                    detail += f' (errno={error.errno})'
                result['errors'].append(detail)
                self.emit(detail)
                return None

        # Drain a bounded number of boot packets, regardless of INT state.
        # No QUICK/SMBus-register scanning and no sensor report configuration.
        for _ in range(16):
            if read() is None:
                break
        else:
            result['errors'].append('Boot drain packet limit reached')
            self.emit(result['errors'][-1])
            result['int_low'] = self.seen_low
            return result

        self.emit('TX Product ID request: channel=2, payload=f9 00')
        try:
            self.t.send(2, b'\xf9\x00')
        except OSError as error:
            detail = f'Product ID write failed: {error} (errno={error.errno})'
            self.emit(detail)
            result['errors'].append(detail)
            result['int_low'] = self.seen_low
            return result

        deadline = self.clock() + 2.0
        for _ in range(40):
            if self.clock() >= deadline:
                break
            self.sleep(0.05)
            item = read()
            if item is None:
                continue
            channel, sequence, payload = item
            if channel == 2 and len(payload) == 16 and payload[0] == 0xF8:
                _, cause, major, minor, part, build, patch, _ = struct.unpack('<BBBBIIHH', payload)
                result['product_id'] = dict(reset_cause=cause, version=f'{major}.{minor}.{patch}',
                                            software_part=part, software_build=build)
                self.emit('VALID SH-2 Product ID: ' + json.dumps(result['product_id']))
                break
        if result['product_id'] is None:
            self.emit('No valid SH-2 Product ID response; this does not prove a defective board')
        result['int_low'] = self.seen_low
        return result


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--confirm-stationary', action='store_true',
                   help='confirm motors disabled and all other IMU/bus clients stopped')
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--addresses', type=lambda s: int(s, 0), nargs='+',
                   choices=(0x4A, 0x4B), default=[0x4A, 0x4B])
    p.add_argument('--gpio-chip', default='auto')
    p.add_argument('--rst-gpio', type=int, default=17)
    p.add_argument('--int-gpio', type=int, default=27)
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not args.confirm_stationary:
        p.error('Require --confirm-stationary: motors disabled, other clients stopped')
    if args.i2c_bus < 0:
        p.error('--i2c-bus must be nonnegative')
    if (args.rst_gpio == args.int_gpio or
            any(v not in range(28) or v in (2, 3) for v in (args.rst_gpio, args.int_gpio))):
        p.error('RST/INT must be distinct header GPIOs 0..27, not I2C GPIO2/3')
    t = None
    code = 2
    try:
        print('BENCH ONLY. Hardware reset at EACH address. No ROS topics or motion commands.', flush=True)
        t = ShtpI2C(args.i2c_bus, args.addresses[0], args.gpio_chip, args.rst_gpio, args.int_gpio)
        def log(text):
            print(text, flush=True)
        diagnostic = Diagnostic(t, log=log)
        diagnostic.emit(f'/dev/i2c-{args.i2c_bus}; chip={t.gpio.path}; '
                        f'RST=GPIO{args.rst_gpio}; INT=GPIO{args.int_gpio}')
        diagnostic.emit('INT sampled at ~1 ms: Linux scheduling can miss short pulses. '
                        'I2C syscall duration depends on kernel timeouts.')
        results = [diagnostic.attempt(a) for a in dict.fromkeys(args.addresses)]
        print('SUMMARY ' + json.dumps(results, ensure_ascii=False), flush=True)
        code = 0 if any(r['product_id'] is not None for r in results) else 1
        print('Product ID success is NOT navigation readiness. '
              'Restart hardware/localization/navigation while stationary before reuse.', flush=True)
    except KeyboardInterrupt:
        print('Interrupted; releasing GPIO/I2C', flush=True)
        code = 130
    except Exception as error:
        print(f'DIAGNOSTIC FAILED: {type(error).__name__}: {error}', flush=True)
    finally:
        if t is not None:
            try:
                # Do not leave sensor held in reset if diagnostic failed midway.
                t.gpio._reset_level(True)
            finally:
                t.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
