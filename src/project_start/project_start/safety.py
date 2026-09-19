"""Pure validation helpers shared by the ROS navigation nodes."""
import math


def fresh(receipt_age, stamp_age, timeout):
    return (math.isfinite(receipt_age) and math.isfinite(stamp_age)
            and 0 <= receipt_age <= timeout and -0.1 <= stamp_age <= timeout)


def variance_ok(values, limit):
    return all(math.isfinite(v) and 0 < v <= limit * limit for v in values)


def validate_route(data):
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('Expected a mapping with a waypoints list')
    points = data['waypoints']
    if not 1 <= len(points) <= 10000:
        raise ValueError('Route must contain 1..10000 waypoints')
    result = []
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            raise ValueError(f'Waypoint {i}: expected mapping')
        values = []
        for key, low, high in [('latitude', -90, 90), ('longitude', -180, 180),
                               ('altitude', -1000, 20000)]:
            value = point.get(key, 0.0 if key == 'altitude' else None)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f'Waypoint {i}: invalid {key}')
            values.append(float(value))
        result.append(tuple(values))
    return result


def check_legs(start, points, max_leg):
    previous = start
    for i, point in enumerate(points):
        if not all(math.isfinite(v) for v in point):
            raise ValueError(f'Waypoint {i}: nonfinite map coordinate')
        distance = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if distance > max_leg:
            raise ValueError(f'Leg {i}: {distance:.1f} m exceeds {max_leg:.1f} m; add GPS points')
        previous = point
