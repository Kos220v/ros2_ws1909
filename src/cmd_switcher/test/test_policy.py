"""No ROS installation needed for mux fail-closed policy tests."""
import math
import pytest
from cmd_switcher.policy import select_source


@pytest.mark.parametrize('mode,mode_age,ages,expected', [
    (1, 0.1, {'manual': 0.1, 'auto': 0.1}, 'manual'),
    (1, 0.1, {'manual': 0.3, 'auto': 0.1}, None),
    (1, 0.1, {'app_manual': 0.1}, 'app_manual'),
    (0, 0.1, {'manual': 0.1, 'auto': 0.1}, 'auto'),
    (0, 0.1, {'auto': 0.4}, None),
    (0, 0.6, {'auto': 0.1}, None),
    (0, math.nan, {'auto': 0.1}, None),
    (0, -1, {'auto': 0.1}, None),
    (0, 0.1, {'auto': math.nan}, None),
    (0, 0.1, {'auto': -1}, None),
    (2, 0.1, {'auto': 0.1}, None),
    (3, 0.1, {'home': 0.1}, None),
    (99, 0.1, {'auto': 0.1}, None),
    (1, 0.1, {}, None)])
def test_selection(mode, mode_age, ages, expected):
    assert select_source(mode, mode_age, ages) == expected
