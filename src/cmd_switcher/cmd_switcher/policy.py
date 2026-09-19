"""Mode selection independent of ROS, fail closed on stale mode heartbeat."""
import math


def select_source(mode, mode_age, ages):
    if not math.isfinite(mode_age) or not 0 <= mode_age < 0.5:
        return None
    # AVOID and RETURN_HOME are deliberately disabled until implemented safely.
    sources = [('manual', 0.2), ('app_manual', 0.3)] if mode == 1 else (
        [('auto', 0.3)] if mode == 0 else [])
    for source, timeout in sources:
        age = ages.get(source, float('inf'))
        if math.isfinite(age) and 0 <= age < timeout:
            return source
    return None
