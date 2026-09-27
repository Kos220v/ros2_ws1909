"""Per-report freshness, not cached-property polling masquerading as new IMU data."""
from .protocol import ACCEL, GYRO, ROTATION


class Samples:
    required = (ACCEL, GYRO, ROTATION)

    def __init__(self, max_age, min_accuracy, max_heading_error):
        self.max_age = max_age
        self.min_accuracy = min_accuracy
        self.max_heading_error = max_heading_error
        self.latest = {}
        self.pending = set()
        self.sequences = {}

    def add(self, report, received):
        previous = self.sequences.get(report.sensor)
        if previous is not None and not 0 < (report.sequence - previous) % 256 < 128:
            return False
        self.sequences[report.sensor] = report.sequence
        self.latest[report.sensor] = (report, received)
        self.pending.add(report.sensor)
        return True

    def take(self, now):
        if not all(sensor in self.pending for sensor in self.required):
            return None
        for sensor in self.required:
            report, received = self.latest[sensor]
            if not 0 <= now - received or now - received + report.delay > self.max_age:
                return None
        rotation = self.latest[ROTATION][0]
        if (rotation.accuracy < self.min_accuracy
                or rotation.heading_accuracy > self.max_heading_error):
            return None
        result = {sensor: self.latest[sensor] for sensor in self.required}
        self.pending.difference_update(self.required)
        return result

    def missing(self, now, timeout, started):
        return [sensor for sensor in self.required
                if now - self.latest.get(sensor, (None, started))[1] > timeout]
