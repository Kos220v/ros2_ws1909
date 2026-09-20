"""Exclusive Linux GPIO character-device access (libgpiod 1.x and 2.x).

Physical levels are used throughout: RST idle HIGH, INT ready LOW.
No sysfs GPIO, pigpio daemon or hard-coded Pi 5 gpiochip number.
"""
import glob
import time


class SensorGPIO:
    def __init__(self, chip='auto', rst=17, interrupt=27):
        if any(type(x) is not int or x < 0 for x in (rst, interrupt)) or rst == interrupt:
            raise ValueError('RST and INT must be distinct nonnegative GPIO line offsets')
        import gpiod
        self.g = gpiod
        self.chip = self.request = self.rst = self.interrupt = None
        self.rst_offset, self.int_offset = rst, interrupt
        self.v2 = hasattr(gpiod, 'request_lines')
        try:
            paths = sorted(glob.glob('/dev/gpiochip*')) if chip == 'auto' else [chip]
            candidates = []
            for path in paths:
                with gpiod.Chip(path) as candidate:
                    label = candidate.get_info().label if self.v2 else candidate.label()
                    if chip != 'auto' or label == 'pinctrl-rp1':
                        candidates.append(path)
            if len(candidates) != 1:
                raise RuntimeError('Cannot uniquely find Pi 5 pinctrl-rp1; set gpio_chip explicitly')
            self.path = candidates[0]
            self.chip = gpiod.Chip(self.path)
            for offset in (rst, interrupt):
                name = (self.chip.get_line_info(offset).name if self.v2
                        else self.chip.get_line(offset).name())
                if name != f'GPIO{offset}':
                    raise ValueError(f'{self.path} line {offset} is {name!r}, not GPIO{offset}')
            if self.v2:
                from gpiod.line import Direction, Value, Bias
                self.request = gpiod.request_lines(self.path, consumer='bno086_imu', config={
                    interrupt: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP),
                    rst: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE),
                })
            else:
                self.interrupt = self.chip.get_line(interrupt)
                self.interrupt.request(consumer='bno086_imu', type=gpiod.LINE_REQ_DIR_IN,
                                       flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
                self.rst = self.chip.get_line(rst)
                self.rst.request(consumer='bno086_imu', type=gpiod.LINE_REQ_DIR_OUT,
                                 default_vals=[1])
        except Exception:
            self.close()
            raise

    def _reset_level(self, high):
        if self.v2:
            from gpiod.line import Value
            self.request.set_value(self.rst_offset, Value.ACTIVE if high else Value.INACTIVE)
        else:
            self.rst.set_value(int(high))

    def ready(self):
        if self.v2:
            from gpiod.line import Value
            return self.request.get_value(self.int_offset) == Value.INACTIVE
        return self.interrupt.get_value() == 0

    def reset(self):
        # Conservative pulse; always release reset even if sleep is interrupted.
        self._reset_level(False)
        try:
            time.sleep(0.01)
        finally:
            self._reset_level(True)
        time.sleep(0.3)

    def wait_ready(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not self.ready():
            if time.monotonic() >= deadline:
                raise TimeoutError('BNO086 INT remained HIGH after hardware reset')
            time.sleep(0.001)

    def close(self):
        # Release only requested lines; idempotent, including partial init failure.
        if self.request is not None:
            self.request.release()
            self.request = None
        for attr in ('rst', 'interrupt'):
            line = getattr(self, attr)
            if line is not None:
                if line.is_requested():
                    line.release()
                setattr(self, attr, None)
        if self.chip is not None:
            self.chip.close()
            self.chip = None
