import math
import pytest
from kolesa_control.tachometer import Tachometer, delta_i32


def tracker(**kwargs):
    return Tachometer(.001, .01, **kwargs)


def test_counter_displacement_not_rpm_or_dt_integration():
    a, b = tracker(), tracker()
    for meter in (a, b):
        meter.update(100, 1000, 1.0)
    a.update(200, 1100, 1.1)
    b.update(200, 1100, 1.4)
    assert a.distance == b.distance == .1
    assert a.travel == b.travel == .1
    assert a.speed == pytest.approx(1.0)
    assert b.speed == pytest.approx(.25)


def test_reverse_and_hardware_total_travel_between_polls():
    t = tracker()
    t.update(100, 1000, 1.)
    t.update(200, 1100, 1.1)
    t.update(100, 1200, 1.2)
    assert t.distance == 0
    assert t.travel == .2
    assert t.speed == pytest.approx(-1)
    # Forward 50, reverse 50 between polls: net signed count doesn't change.
    t.update(100, 1300, 1.3)
    assert t.distance == 0
    assert t.travel == .3  # not sum(abs(observed signed deltas)) == .2


def test_encoder_sign_and_mechanical_angle():
    t = tracker(sign=-1)
    t.update(0, 0, 1.)
    t.update(-100, 100, 1.1)
    assert t.distance == .1 and t.angle == 1.
    assert t.travel == .1


def test_counter_rollover_both_directions_and_absolute():
    assert delta_i32(-2147483648, 2147483647) == 1
    assert delta_i32(2147483647, -2147483648) == -1
    t = tracker()
    t.update(2147483640, 2147483640, 1.)
    t.update(-2147483646, -2147483646, 1.1)
    assert t.counts == t.travel_counts == 10
    t.update(2147483640, -2147483636, 1.2)
    assert t.counts == 0 and t.travel_counts == 20
    u = tracker()
    u.update(100, -6, 1.)
    u.update(110, 4, 1.1)
    assert u.travel_counts == 10


def test_total_larger_than_int32_not_modulo_initial_counter():
    t = tracker(max_speed=1e9)
    signed, unsigned = 0, 0
    t.update(0, 0, 1.)
    for i in range(1, 101):
        signed = (signed + 100_000_000 + (1<<31)) % (1<<32) - (1<<31)
        unsigned = (unsigned + 100_000_000 + (1<<31)) % (1<<32) - (1<<31)
        t.update(signed, unsigned, 1. + i*.1)
    assert t.counts == t.travel_counts == 10_000_000_000
    assert t.distance == 10_000_000


def test_missed_packets_do_not_lose_counted_distance():
    sparse, dense = tracker(), tracker()
    sparse.update(0, 0, 1.)
    dense.update(0, 0, 1.)
    for i in range(1, 5):
        dense.update(i*10, i*10, 1+i*.05)
    sparse.update(40, 40, 1.2)
    assert sparse.distance == dense.distance == .04
    assert sparse.travel == dense.travel == .04


@pytest.mark.parametrize('signed,absolute', [(100000, 100000), (0, 0), (110, 1001)])
def test_jump_reset_or_inconsistent_pair_latches_and_does_not_mutate(signed, absolute):
    t = tracker()
    t.update(100, 1000, 1.)
    t.update(105, 1005, 1.1)
    before = (t.counts, t.travel_counts, t.previous, t.previous_abs, t.received)
    assert not t.update(signed, absolute, 1.2)
    assert t.fault and not t.velocity_valid
    assert (t.counts, t.travel_counts, t.previous, t.previous_abs, t.received) == before
    assert not t.update(110, 1010, 1.3)  # requires explicit node/localization restart
    assert t.speed == t.omega == 0


def test_gap_preserves_distance_but_requires_new_interval_for_velocity():
    t = tracker()
    t.update(0, 0, 1.)
    t.update(100, 100, 3.)
    assert t.distance == .1
    assert not t.velocity_valid and t.speed == 0
    t.update(110, 110, 3.1)
    assert t.velocity_valid and t.speed == pytest.approx(.1)


@pytest.mark.parametrize('stamp', [1., .9])
def test_duplicate_out_of_order_ignored(stamp):
    t = tracker()
    t.update(0, 0, 1.)
    assert not t.update(100, 100, stamp)
    assert t.distance == 0 and t.received == 1.


@pytest.mark.parametrize('value', [None, True, 1.2, math.nan, 1<<31, -(1<<31)-1])
def test_invalid_counter_rejected(value):
    with pytest.raises(ValueError):
        tracker().update(value, 0, 1.)


@pytest.mark.parametrize('scale', [0, -1, math.nan, math.inf])
def test_invalid_calibration(scale):
    with pytest.raises(ValueError):
        Tachometer(scale, .01)
