"""Test actual hardware-node publication method with lightweight message/clock fakes."""
import ast
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace as NS
from kolesa_control.tachometer import Tachometer


class Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return NS(nanoseconds=self.nanoseconds)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))


def node():
    path = Path(__file__).parents[1] / 'kolesa_control/kolesa_control_node.py'
    defs = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef)]
    scope = dict(Node=object, Twist=object, Time=Clock, math=math,
                 time=NS(monotonic=lambda: 20.), TrackTicks=lambda: NS(header=NS()))
    exec(compile(ast.Module(body=defs, type_ignores=[]), str(path), 'exec'), scope)
    obj = object.__new__(scope['KolesaControl'])
    obj.get_clock = lambda: NS(now=lambda: Clock(20_000_000_000))
    obj.wheels = {side:obj._make_wheel_state() for side in ('left','right')}
    obj.trackers = {side:Tachometer(.001, .01) for side in ('left','right')}
    for side, stamp in (('left',19.95),('right',19.97)):
        obj.trackers[side].update(0,0,stamp-.1)
        obj.trackers[side].update(10,10,stamp)
        obj.wheels[side].update(measurement_valid=True, counter_valid=True,
                               last_rx_time=stamp, stale=False)
    obj.last_track_stamps = {'left':None,'right':None}
    obj.track_pubs = {'left':Publisher(),'right':Publisher()}
    obj.odometry_session, obj.base_frame = 'boot-session', 'base_link'
    obj.telemetry_stale_timeout, obj.telemetry_pair_max_skew = .5, .1
    return obj


def test_per_track_measurement_stamp_session_and_scale():
    obj = node()
    obj._publish_track_samples()
    left = obj.track_pubs['left'].messages[-1]
    right = obj.track_pubs['right'].messages[-1]
    assert abs(left.header.stamp.nanoseconds-19_950_000_000) <= 1
    assert abs(right.header.stamp.nanoseconds-19_970_000_000) <= 1
    assert left.header.stamp.nanoseconds != right.header.stamp.nanoseconds
    assert left.session_id == right.session_id == 'boot-session'
    assert left.position_ticks == right.position_ticks == 10
    assert left.meters_per_tick == right.meters_per_tick == .001
    assert left.valid and right.valid


def test_cached_counters_not_published_with_new_stamp():
    obj = node()
    obj._publish_track_samples()
    obj._publish_track_samples()
    assert len(obj.track_pubs['left'].messages) == 1
    assert len(obj.track_pubs['right'].messages) == 1


def test_invalid_track_explicitly_invalid_not_zero_travel():
    obj = node()
    obj.wheels['left']['measurement_valid'] = False
    obj._publish_track_samples()
    msg = obj.track_pubs['left'].messages[-1]
    assert not msg.valid and msg.position_ticks == 10


def test_command_gate_rechecks_freshness_without_telemetry_timer():
    obj = node()
    assert obj._both_tracks_valid()
    obj.wheels['left']['last_rx_time'] = 19.
    assert not obj._both_tracks_valid()
    obj.wheels['left']['last_rx_time'] = 21.
    assert not obj._both_tracks_valid()
