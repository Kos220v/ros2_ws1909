from dataclasses import replace
from bno086_imu.protocol import ACCEL, GYRO, ROTATION, Report
from bno086_imu.samples import Samples


def report(sensor, seq=1):
    values = (0, 0, 0, 1) if sensor == ROTATION else (0, 0, 0)
    return Report(sensor, seq, 3, 0, values, 0.1)


def populate(samples, now=1.0, seq=1):
    for sensor in (ACCEL, GYRO, ROTATION):
        samples.add(report(sensor, seq), now)


def test_no_data_no_publish():
    assert Samples(.1, 2, .35).take(1.0) is None


def test_once_per_fresh_bundle():
    samples = Samples(.1, 2, .35)
    populate(samples)
    assert samples.take(1.01) is not None
    assert samples.take(1.02) is None
    assert not samples.add(report(ROTATION), 1.03)
    assert samples.take(1.03) is None
    samples.add(report(ROTATION, 2), 1.04)
    assert samples.take(1.05) is None  # old gyro/accel may not be re-stamped
    samples.add(report(GYRO, 2), 1.05)
    samples.add(report(ACCEL, 2), 1.05)
    assert samples.take(1.06) is not None


def test_stale_missing_component_and_watchdog():
    samples = Samples(.1, 2, .35)
    populate(samples)
    samples.add(report(ROTATION, 2), 1.3)
    assert samples.take(1.31) is None
    assert samples.missing(3.1, 2.0, 1.0) == [ACCEL, GYRO]
    assert Samples(.1,2,.35).missing(3.1,2.0,1.0) == [ACCEL, GYRO, ROTATION]


def test_bad_accuracy_waits_without_resetting_calibration():
    samples = Samples(.1, 2, .35)
    populate(samples)
    samples.add(replace(report(ROTATION,2), accuracy=0), 1.02)
    assert samples.take(1.03) is None
    assert samples.missing(1.03, 2.0, 1.0) == []
    samples.add(report(ROTATION,3), 1.04)
    assert samples.take(1.05) is not None


def test_large_heading_uncertainty():
    samples = Samples(.1, 2, .35)
    populate(samples)
    samples.add(replace(report(ROTATION,2), heading_accuracy=.5), 1.02)
    assert samples.take(1.03) is None


def test_delay_added_to_receipt_age():
    samples = Samples(.1, 2, .35)
    populate(samples)
    samples.add(replace(report(GYRO,2), delay=.09), 1.0)
    assert samples.take(1.02) is None


def test_counter_wrap_replay_and_out_of_order():
    samples = Samples(.1, 2, .35)
    assert samples.add(report(ROTATION,255), 1.)
    assert samples.add(report(ROTATION,0), 1.01)
    assert not samples.add(report(ROTATION,255), 1.02)
    assert not samples.add(report(ROTATION,0), 10.)
    assert samples.latest[ROTATION][1] == 1.01


def test_clock_backwards():
    samples = Samples(.1,2,.35)
    populate(samples)
    assert samples.take(.9) is None
