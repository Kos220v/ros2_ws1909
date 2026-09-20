"""Exercise both supported libgpiod APIs without touching a GPIO device."""
import sys
from enum import Enum
from types import SimpleNamespace
import pytest
from bno086_imu.gpio import SensorGPIO


@pytest.fixture(params=[1, 2])
def backend(request, monkeypatch):
    version = request.param
    state = SimpleNamespace(levels={17: 1, 27: 0}, held=set(), sets=[], fail=False,
                            label='pinctrl-rp1', names={17: 'GPIO17', 27: 'GPIO27'})
    class Line:
        def __init__(self, offset): self.offset = offset
        def name(self): return state.names[self.offset]
        def request(self, **kw):
            if state.fail and self.offset == 17: raise OSError('busy')
            state.held.add(self.offset)
            if 'default_vals' in kw:
                state.levels[self.offset] = kw['default_vals'][0]
            else:
                assert kw['flags'] == 4  # input pull-up
        def is_requested(self): return self.offset in state.held
        def release(self): state.held.remove(self.offset)
        def get_value(self): return state.levels[self.offset]
        def set_value(self, value):
            state.levels[self.offset] = value
            state.sets.append(value)
    class Chip:
        def __init__(self, path): pass
        def __enter__(self): return self
        def __exit__(self, *args): self.close()
        def close(self): pass
        def label(self): return state.label
        def get_info(self): return SimpleNamespace(label=state.label)
        def get_line(self, offset): return Line(offset)
        def get_line_info(self, offset): return SimpleNamespace(name=state.names[offset])
    class Value(Enum):
        INACTIVE = 0
        ACTIVE = 1
    class Request:
        def get_value(self, offset): return Value(state.levels[offset])
        def set_value(self, offset, value): Line(offset).set_value(value.value)
        def release(self): state.held.clear()
    def request_lines(path, consumer, config):
        if state.fail: raise OSError('busy')
        assert config[27].bias == 'up'
        assert config[17].output_value == Value.ACTIVE
        state.held.update(config)
        return Request()
    api = SimpleNamespace(Chip=Chip, LINE_REQ_DIR_IN=1, LINE_REQ_DIR_OUT=2,
                          LINE_REQ_FLAG_BIAS_PULL_UP=4)
    if version == 2:
        api.request_lines = request_lines
        api.LineSettings = SimpleNamespace
    monkeypatch.setitem(sys.modules, 'gpiod', api)
    monkeypatch.setitem(sys.modules, 'gpiod.line', SimpleNamespace(
        Value=Value, Direction=SimpleNamespace(INPUT=1, OUTPUT=2),
        Bias=SimpleNamespace(PULL_UP='up')))
    monkeypatch.setattr('bno086_imu.gpio.glob.glob', lambda _: ['/dev/gpiochip7'])
    return state


def test_discovery_reset_levels_and_release(backend, monkeypatch):
    delays = []
    monkeypatch.setattr('bno086_imu.gpio.time.sleep', delays.append)
    gpio = SensorGPIO()
    assert gpio.path == '/dev/gpiochip7'
    assert backend.held == {17, 27}
    assert backend.levels[17] == 1
    assert gpio.ready()
    gpio.wait_ready()
    backend.levels[27] = 1
    assert not gpio.ready()
    gpio.reset()
    assert backend.sets == [0, 1]
    assert delays == [.01, .3]
    gpio.close()
    gpio.close()
    assert not backend.held


def test_busy_releases_partial_acquisition(backend):
    backend.fail = True
    with pytest.raises(OSError, match='busy'):
        SensorGPIO()
    assert not backend.held


def test_wrong_chip_or_line_rejected(backend):
    backend.label = 'wrong controller'
    with pytest.raises(RuntimeError, match='pinctrl-rp1'):
        SensorGPIO()
    backend.label = 'pinctrl-rp1'
    backend.names[17] = 'unrelated signal'
    with pytest.raises(ValueError, match='not GPIO17'):
        SensorGPIO()
    assert not backend.held


def test_missing_int_timeout(backend, monkeypatch):
    gpio = SensorGPIO()
    backend.levels[27] = 1
    clock = iter([0., 3.])
    monkeypatch.setattr('bno086_imu.gpio.time.monotonic', lambda: next(clock))
    with pytest.raises(TimeoutError, match='INT remained HIGH'):
        gpio.wait_ready(2.)
    gpio.close()


def test_reset_released_on_interrupted_sleep(backend, monkeypatch):
    gpio = SensorGPIO()
    def interrupted(_): raise KeyboardInterrupt()
    monkeypatch.setattr('bno086_imu.gpio.time.sleep', interrupted)
    with pytest.raises(KeyboardInterrupt):
        gpio.reset()
    assert backend.sets == [0, 1]
    gpio.close()


@pytest.mark.parametrize('rst,interrupt', [(17,17),(-1,27),(True,27)])
def test_invalid_offsets(rst, interrupt):
    with pytest.raises(ValueError):
        SensorGPIO(rst=rst, interrupt=interrupt)
