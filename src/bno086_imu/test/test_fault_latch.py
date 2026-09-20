"""Execute actual node safety methods independently of unavailable ROS bindings."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'bno086_imu/node.py'


def method(name):
    cls = next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n, ast.ClassDef))
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = {'math': math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), ns)
    return ns[name]


def test_fault_stops_all_subsequent_polling():
    calls = []
    def poll():
        calls.append('poll')
        raise TimeoutError('INT/report timeout')
    node = SimpleNamespace(fault_reason=None, _poll=poll,
        close=lambda: calls.append('close'), diagnostics=lambda: calls.append('diagnostics'),
        get_logger=lambda: SimpleNamespace(error=lambda _: calls.append('error')))
    method('poll')(node)
    assert node.fault_reason == 'INT/report timeout'
    assert calls == ['poll', 'error', 'close', 'diagnostics']
    method('poll')(node)
    assert calls == ['poll', 'error', 'close', 'diagnostics']


@pytest.mark.parametrize('rst,interrupt', [(17,17),(2,27),(17,3),(-1,27),(28,27),(True,27)])
def test_invalid_gpio_configuration_fails_before_hardware(rst, interrupt):
    node = SimpleNamespace(p=dict(gpio_chip='auto', rst_gpio=rst, int_gpio=interrupt))
    with pytest.raises(ValueError, match='RST/INT'):
        method('validate')(node)
