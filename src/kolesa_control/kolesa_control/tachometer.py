"""Distance from VESC hardware counters, never from integration of RPM/velocity.

The Python integer totals unwrap adjacent 32-bit counter differences; time is
used ONLY for plausibility checks and the derivative (velocity). A discontinuity
latches a fault rather than silently inventing displacement across a VESC reset.
"""
import math


def delta_i32(current, previous):
    return ((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)


def wire_counter(value):
    if type(value) is not int or not -(1 << 31) <= value < (1 << 31):
        raise ValueError('tachometers must be signed int32 wire values')
    return value


class Tachometer:
    def __init__(self, meters_per_count, radians_per_count, sign=1,
                 max_speed=1.24, jump_margin=3.0, min_jump_counts=10,
                 stale_timeout=0.5):
        for value in (meters_per_count, radians_per_count, max_speed, jump_margin, stale_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('scale, speed, margin and timeout must be finite and positive')
        if not math.isfinite(min_jump_counts) or min_jump_counts < 0 or sign not in (-1, 1):
            raise ValueError('invalid counter threshold or encoder sign')
        self.meters_per_count = meters_per_count
        self.radians_per_count = radians_per_count
        self.sign = sign
        self.max_speed = max_speed
        self.jump_margin = jump_margin
        self.min_jump_counts = min_jump_counts
        self.stale_timeout = stale_timeout
        self.previous = self.previous_abs = self.initial = None
        self.received = None
        self.counts = self.travel_counts = self.delta_counts = 0
        self.speed = self.omega = 0.0
        self.velocity_valid = False
        self.fault = ''

    @property
    def distance(self):
        return self.counts * self.meters_per_count

    @property
    def travel(self):
        return self.travel_counts * self.meters_per_count

    @property
    def angle(self):
        # odometry_scale describes ground distance, not mechanical shaft angle.
        return self.counts * self.radians_per_count

    def update(self, signed, absolute, received):
        signed, absolute = wire_counter(signed), wire_counter(absolute)
        if not math.isfinite(received) or received < 0:
            raise ValueError('invalid monotonic receipt time')
        if self.fault or (self.received is not None and received <= self.received):
            return False
        if self.received is None:
            self.initial = self.previous = signed
            self.previous_abs = absolute
            self.received = received
            return True
        dt = received - self.received
        delta = delta_i32(signed, self.previous)
        # tachometer_abs counts travel in either direction. Wire representation
        # is int32, but its difference is unsigned modulo 2**32.
        travel_delta = (absolute - self.previous_abs) % (1 << 32)
        limit = max(self.min_jump_counts,
                    self.max_speed * dt * self.jump_margin / self.meters_per_count)
        if abs(delta) > limit or travel_delta > limit or travel_delta + 2 < abs(delta):
            self.fault = 'counter discontinuity/reset: stop and restart localization at standstill'
            self.velocity_valid = False
            self.speed = self.omega = 0.0
            self.delta_counts = 0
            return False
        # Commit ONLY after validating both hardware counters. This is exact
        # integer counter unwrapping, not numerical velocity integration.
        self.counts += self.sign * delta
        self.travel_counts += travel_delta
        self.delta_counts = self.sign * delta
        self.previous, self.previous_abs, self.received = signed, absolute, received
        self.velocity_valid = dt <= self.stale_timeout
        self.speed = self.delta_counts * self.meters_per_count / dt if self.velocity_valid else 0.0
        self.omega = self.delta_counts * self.radians_per_count / dt if self.velocity_valid else 0.0
        return True
