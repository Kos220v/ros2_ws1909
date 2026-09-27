"""Analytical trajectories and timing/failure regression without ROS hardware."""
import math
import numpy as np
import pytest
from project_start.odometry_math import (
    CounterOdometry, OdometryError, Series, arc, wrap, body_orientation_and_rate,
)


def sample(model, t, left, right, yaw, rate=0.0, scale=1e-6):
    model.push_track('left', t, round(left/scale), scale, 'session')
    model.push_track('right', t, round(right/scale), scale, 'session')
    model.push_imu(t, wrap(yaw), .0025, rate, .0004)
    return model.advance()


def test_arc_straight_reverse_quarter_and_small_angle():
    assert arc(2, 0, 0) == (2, 0)
    assert arc(-2, 0, 0) == (-2, 0)
    assert arc(math.pi/2, 0, math.pi/2) == pytest.approx((1, 1))
    assert arc(-math.pi/2, 0, math.pi/2) == pytest.approx((-1, -1))
    assert arc(1, 0, 1e-10) == pytest.approx((1, 5e-11))


@pytest.mark.parametrize('heading', [0, math.pi/2, math.pi, -math.pi/2])
def test_straight_uses_counter_displacement_and_absolute_enu_heading(heading):
    m = CounterOdometry()
    assert sample(m, 1., 0, 0, heading) is None
    for i in range(1, 101):
        r = sample(m, 1+i*.05, i*.05, i*.05, heading)
    assert r.x == pytest.approx(5*math.cos(heading), abs=1e-7)
    assert r.y == pytest.approx(5*math.sin(heading), abs=1e-7)
    assert r.vx == pytest.approx(1.)
    assert r.yaw == pytest.approx(heading)


def test_reverse_and_pivot_no_fake_translation():
    m = CounterOdometry()
    sample(m, 1, 0, 0, 0)
    r = sample(m, 1.1, -.1, -.1, 0)
    assert r.x == pytest.approx(-.1)
    r = sample(m, 1.2, -.124, -.076, .1, rate=1.)
    assert (r.x, r.y) == pytest.approx((-.1, 0))
    assert r.yaw == pytest.approx(.1)
    assert r.wz == 1.


def test_complete_circle_with_timestamp_skew_and_different_sensor_rates():
    m = CounterOdometry()
    radius, speed, rate = 2., 1., .5
    # Independent UARTs: right stream offset by 17ms. IMU at 25Hz, VESC at 20Hz.
    events = [(i*.05, 'left') for i in range(260)]
    events += [(i*.05+.017, 'right') for i in range(259)]
    events += [(i*.04, 'imu') for i in range(325)]
    outputs = []
    for t, key in sorted(events):
        if key == 'imu':
            m.push_imu(t+1, wrap(rate*t), .0025, rate, .0004)
        else:
            v = speed + (.24*rate if key == 'right' else -.24*rate)
            m.push_track(key, t+1, round(v*t*1e6), 1e-6, 'session')
        r = m.advance()
        if r:
            outputs.append(r)
    r = outputs[-1]
    start, end = .017, r.stamp-1
    expected = (radius*(math.sin(rate*end)-math.sin(rate*start)),
                radius*(math.cos(rate*start)-math.cos(rate*end)))
    assert (r.x, r.y) == pytest.approx(expected, abs=2e-5)
    assert r.yaw == pytest.approx(wrap(rate*end))
    assert r.vx == pytest.approx(1., abs=1e-4)
    assert np.min(np.linalg.eigvalsh(r.covariance)) >= -1e-10
    assert np.allclose(r.covariance, r.covariance.T)


def test_variable_heading_split_at_intermediate_imu_samples():
    m = CounterOdometry(max_gap=.3, max_track_speed=10, max_yaw_rate=10)
    sample(m, 1, 0, 0, 0)
    # Each half has different curvature; integrating one chord would be wrong.
    m.push_track('left', 1.2, 200000, 1e-6, 'session')
    m.push_track('right', 1.2, 200000, 1e-6, 'session')
    m.push_imu(1.1, .5, .0025, 5., .0004)
    m.push_imu(1.2, .5, .0025, 0., .0004)
    r = m.advance()
    a, b = arc(.1, 0, .5), arc(.1, .5, .5)
    assert (r.x, r.y) == pytest.approx((a[0]+b[0], a[1]+b[1]))


def test_stationary_does_not_integrate_gyro_bias_into_position_or_heading():
    m = CounterOdometry()
    sample(m, 1, 0, 0, .4, rate=.02)
    for i in range(1, 200):
        r = sample(m, 1+i*.05, 0, 0, .4, rate=.02)
    assert (r.x, r.y, r.yaw) == pytest.approx((0, 0, .4))
    assert r.covariance[0, 0] == pytest.approx(1e-6)


def test_yaw_wrap_is_not_a_360_degree_turn():
    m = CounterOdometry()
    sample(m, 1, 0, 0, math.pi-.02)
    r = sample(m, 1.1, .1, .1, -math.pi+.02, rate=.4)
    assert r.x < -.099
    assert abs(r.y) < 1e-6


def test_dropout_is_not_extrapolated():
    m = CounterOdometry()
    sample(m, 1, 0, 0, 0)
    with pytest.raises(OdometryError, match='gap'):
        sample(m, 1.3, .1, .1, 0)
    assert (m.x, m.y) == (0, 0)


def test_old_or_duplicate_samples_do_not_create_new_motion():
    m = CounterOdometry()
    sample(m, 1, 0, 0, 0)
    assert not m.push_track('left', 1, 100, 1e-6, 'session')
    assert m.advance() is None
    with pytest.raises(OdometryError, match='backwards'):
        m.push_imu(.9, 0, .01, 0, .01)


def test_sessions_and_calibration_cannot_silently_reset():
    m = CounterOdometry()
    sample(m, 1, 0, 0, 0)
    with pytest.raises(OdometryError, match='session'):
        m.push_track('left', 1.1, 0, 1e-6, 'new')
    with pytest.raises(OdometryError, match='calibration'):
        m.push_track('right', 1.1, 0, .002, 'session')


def test_counter_and_heading_jumps():
    m = CounterOdometry()
    sample(m, 1, 0, 0, 0)
    with pytest.raises(OdometryError, match='movement'):
        m.push_track('left', 1.05, 1000000, 1e-6, 'session')
    with pytest.raises(OdometryError, match='heading jump'):
        m.push_imu(1.05, 1., .01, 0, .01)


def test_slip_inflates_uncertainty_without_replacing_imu_yaw():
    straight, slipping = CounterOdometry(), CounterOdometry()
    for m in (straight, slipping):
        sample(m, 1, 0, 0, 0)
    a = sample(straight, 1.1, .1, .1, 0)
    b = sample(slipping, 1.1, .05, .15, 0)
    assert (a.x, a.y, a.yaw) == pytest.approx((b.x, b.y, b.yaw))
    assert b.covariance[0, 0] > a.covariance[0, 0]
    assert b.slip_residual > a.slip_residual


def test_mount_rotation_and_tilt():
    s = math.sqrt(.5)
    # Sensor installed +90deg around base Z; base itself faces true east.
    yaw, rate = body_orientation_and_rate((0,0,s,s), (0,0,s,s), (0,0,1), .35)
    assert (yaw, rate) == pytest.approx((0, 1))
    with pytest.raises(OdometryError, match='tilt'):
        body_orientation_and_rate((s,0,0,s), (0,0,0,1), (0,0,0), .35)


def test_series_no_extrapolation_and_bounded_preflight_history():
    series = Series(.15)
    for i in range(2000):
        series.add(i*.05, [float(i)], False)
    assert len(series.times) < 12
    assert series.at(series.times[-1])[0] == 1999
    with pytest.raises(OdometryError, match='extrapolation'):
        series.at(1000)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1])
def test_invalid_variance(bad):
    with pytest.raises(OdometryError):
        CounterOdometry().push_imu(1, 0, bad, 0, .01)
