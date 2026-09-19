"""Timestamp-aligned counter + absolute-heading odometry, independent of ROS.

Planar no-lateral-slip model; exact SE(2) arc for each piecewise constant-curvature
segment. A hardware counter is NOT an absolute Cartesian position sensor.
"""
from bisect import bisect_left
from dataclasses import dataclass
import math
import numpy as np


class OdometryError(ValueError):
    """Data continuity or physical plausibility cannot be guaranteed."""


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sinc_and_derivative(x):
    if abs(x) < 1e-4:
        return 1 - x*x/6 + x**4/120, -x/3 + x**3/30
    return math.sin(x)/x, (x*math.cos(x)-math.sin(x))/(x*x)


def arc(ds, heading0, heading1):
    half = (heading1-heading0)/2
    scale, _ = sinc_and_derivative(half)
    mid = (heading0+heading1)/2
    return ds*scale*math.cos(mid), ds*scale*math.sin(mid)


class Series:
    def __init__(self, max_gap):
        self.max_gap = max_gap
        self.times, self.values = [], []

    def add(self, stamp, values, active):
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in values):
            raise OdometryError('nonfinite sample')
        if self.times:
            dt = stamp-self.times[-1]
            if dt == 0:
                return False
            if dt < 0:
                raise OdometryError('timestamps moved backwards')
            if dt > self.max_gap + 1e-9:
                if active:
                    raise OdometryError('sample gap: unknown motion during missing data')
                self.times.clear()
                self.values.clear()
        self.times.append(stamp)
        self.values.append(np.array(values, dtype=float))
        if not active:
            while len(self.times) > 2 and self.times[1] < stamp - 3*self.max_gap:
                del self.times[0]
                del self.values[0]
        if len(self.times) > 512:
            raise OdometryError('input backlog')
        return True

    def at(self, stamp):
        i = bisect_left(self.times, stamp)
        if i < len(self.times) and abs(self.times[i]-stamp) < 1e-9:
            return self.values[i].copy()
        if i == 0 or i == len(self.times):
            raise OdometryError('extrapolation is forbidden')
        a, b = self.times[i-1], self.times[i]
        if b-a > self.max_gap + 1e-9:
            raise OdometryError('interpolation gap too large')
        f = (stamp-a)/(b-a)
        return self.values[i-1]*(1-f) + self.values[i]*f

    def prune(self, stamp):
        while len(self.times) > 2 and self.times[1] <= stamp:
            del self.times[0]
            del self.values[0]


@dataclass
class Estimate:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float
    covariance: np.ndarray
    vx_variance: float
    wz_variance: float
    slip_residual: float


class CounterOdometry:
    def __init__(self, max_gap=0.15, max_track_speed=2.0, max_yaw_rate=2.0,
                 yaw_jump_tolerance=0.08, track_width=0.48,
                 distance_stddev_per_meter=0.03, distance_noise_floor=0.001,
                 lateral_stddev_per_meter=0.03, lateral_stddev_per_radian=0.02,
                 slip_noise_gain=0.3, heading_bias_stddev=0.05,
                 timestamp_stddev=0.005, min_update_interval=0.02):
        config = locals().copy()
        config.pop('self')
        for name, value in config.items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + ' must be finite and nonnegative')
            setattr(self, name, value)
        if min(max_gap, max_track_speed, max_yaw_rate, track_width, distance_noise_floor) <= 0:
            raise ValueError('gap, physical limits, width and noise floor must be positive')
        self.left, self.right, self.imu = (Series(max_gap) for _ in range(3))
        self.series = (self.left, self.right, self.imu)
        self.sessions, self.scales = {}, {}
        self.time = None
        self.x = self.y = self.yaw = 0.0
        self.P = np.diag([1e-6, 1e-6, 1e-6])

    def push_track(self, side, stamp, ticks, meters_per_tick, session):
        if side not in ('left', 'right') or not session:
            raise OdometryError('track side/session invalid')
        if type(ticks) is not int or not math.isfinite(meters_per_tick) or meters_per_tick <= 0:
            raise OdometryError('invalid track counter/calibration')
        if side in self.sessions and (session != self.sessions[side] or meters_per_tick != self.scales[side]):
            raise OdometryError('counter session/calibration changed; restart localization at standstill')
        other = 'right' if side == 'left' else 'left'
        if other in self.sessions and self.sessions[other] != session:
            raise OdometryError('tracks belong to different hardware sessions')
        self.sessions[side], self.scales[side] = session, meters_per_tick
        series = getattr(self, side)
        distance = ticks*meters_per_tick
        if series.times and stamp > series.times[-1]:
            dt = stamp-series.times[-1]
            delta = distance-series.values[-1][0]
            if abs(delta) > self.max_track_speed*dt + 2*meters_per_tick:
                raise OdometryError('physically implausible track movement')
        return series.add(stamp, [distance], self.time is not None)

    def push_imu(self, stamp, yaw, yaw_variance, yaw_rate, rate_variance):
        values = (yaw, yaw_variance, yaw_rate, rate_variance)
        if not all(math.isfinite(v) for v in values) or min(yaw_variance, rate_variance) <= 0:
            raise OdometryError('invalid IMU/covariance')
        if abs(yaw_rate) > self.max_yaw_rate:
            raise OdometryError('IMU angular velocity exceeds physical limit')
        if self.imu.times:
            dt = stamp-self.imu.times[-1]
            previous = self.imu.values[-1][0]
            delta = wrap(yaw-previous)
            if dt > 0 and abs(delta) > self.max_yaw_rate*dt + self.yaw_jump_tolerance:
                raise OdometryError('IMU heading jump / magnetic correction')
            yaw = previous+delta
        return self.imu.add(stamp, [yaw, yaw_variance, yaw_rate, rate_variance], self.time is not None)

    def advance(self):
        if not all(s.times for s in self.series):
            return None
        end = min(s.times[-1] for s in self.series)
        if self.time is None:
            start = max(s.times[0] for s in self.series)
            if start > end:
                return None
            self.time = start
            self.yaw = self.imu.at(start)[0]
            self.P[2, 2] = self.imu.at(start)[1]
        if end <= self.time + 1e-9 or end-self.time < self.min_update_interval-1e-9:
            return None
        start = self.time
        l0, r0 = self.left.at(start)[0], self.right.at(start)[0]
        # Split at all IMU AND track measurement times, not just at publication
        # ticks. Thus varying curvature/speed doesn't get replaced by one chord.
        knots = sorted({t for s in self.series for t in s.times if start < t <= end} | {end})
        variance_distance = 0.0
        residual = 0.0
        for t in knots:
            dl = self.left.at(t)[0]-self.left.at(self.time)[0]
            dr = self.right.at(t)[0]-self.right.at(self.time)[0]
            previous_heading = self.imu.at(self.time)[0]
            heading = self.imu.at(t)
            ds, dtheta = (dl+dr)/2, heading[0]-previous_heading
            dt = t-self.time
            if dt <= 1e-9:
                continue
            residual += abs((dr-dl) - self.track_width*dtheta)
            slip = abs((dr-dl) - self.track_width*dtheta)
            moving = abs(dl)+abs(dr) > 1e-12
            q_distance = (
                self.distance_noise_floor**2
                + (self.distance_stddev_per_meter*abs(ds))**2
                + (self.slip_noise_gain*slip)**2
                + (abs(ds/dt)*self.timestamp_stddev)**2
            ) if moving else 0.0
            q_lateral = ((self.lateral_stddev_per_meter*abs(ds))**2
                         + (self.lateral_stddev_per_radian*abs(dtheta))**2) if moving else 0.0
            self._step(ds, previous_heading, heading[0], q_distance, q_lateral, heading[1])
            variance_distance += q_distance
            self.time = t
        dt = end-start
        distance = (self.left.at(end)[0]-l0 + self.right.at(end)[0]-r0)/2
        imu = self.imu.at(end)
        # Shared heading bias doesn't average away with more samples.
        bias_jacobian = np.array([-self.y, self.x, 1.0])
        cov = self.P + self.heading_bias_stddev**2 * np.outer(bias_jacobian, bias_jacobian)
        result = Estimate(end, self.x, self.y, wrap(self.yaw), distance/dt, imu[2], cov,
                          max(1e-4, variance_distance/(dt*dt)), imu[3], residual)
        for series in self.series:
            series.prune(end)
        return result

    def _step(self, ds, a, b, q_distance, q_lateral, yaw_variance):
        half, mid = (b-a)/2, (a+b)/2
        g, gp = sinc_and_derivative(half)
        c, s = math.cos(mid), math.sin(mid)
        dx, dy = ds*g*c, ds*g*s
        # State [x,y,old absolute yaw]; measurement [ds,new absolute yaw].
        # New yaw replaces old yaw: no gyro/yaw random walk added twice.
        F = np.array([[1, 0, ds*(-gp*c-g*s)/2],
                      [0, 1, ds*(-gp*s+g*c)/2], [0, 0, 0]], dtype=float)
        G = np.array([[g*c, ds*(gp*c-g*s)/2],
                      [g*s, ds*(gp*s+g*c)/2], [0, 1]], dtype=float)
        lateral = np.array([-s, c, 0.0])
        self.P = F@self.P@F.T + G@np.diag([q_distance, yaw_variance])@G.T
        self.P += q_lateral*np.outer(lateral, lateral)
        self.P = (self.P+self.P.T)/2
        self.x += dx
        self.y += dy
        self.yaw = b


def quaternion_product(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s)


def normalize_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise OdometryError('invalid quaternion')
    norm = math.sqrt(sum(v*v for v in q))
    if not 0.9 <= norm <= 1.1:
        raise OdometryError('quaternion norm invalid')
    return tuple(v/norm for v in q)


def body_orientation_and_rate(sensor_q, mount_q, angular_velocity, max_tilt):
    """q_world_base = q_world_sensor * inverse(q_base_sensor); no tf2 msg plugin needed."""
    q = normalize_quaternion(sensor_q)
    mount = normalize_quaternion(mount_q)
    inverse = (-mount[0], -mount[1], -mount[2], mount[3])
    x, y, z, w = quaternion_product(q, inverse)
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
    if max(abs(roll), abs(pitch)) > max_tilt:
        raise OdometryError('tilt exceeds planar-model limit')
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    if not all(math.isfinite(v) for v in angular_velocity):
        raise OdometryError('nonfinite gyro')
    body_rate = quaternion_product(quaternion_product(mount, (*angular_velocity, 0.0)), inverse)
    # Euler yaw rate, rather than body wz, for a slightly tilted platform.
    yaw_rate = (math.sin(roll)*body_rate[1] + math.cos(roll)*body_rate[2])/math.cos(pitch)
    return yaw, yaw_rate
