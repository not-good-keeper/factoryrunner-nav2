"""Per-leg trial metrics. No ROS imports: the trial runner feeds plain numbers in."""

import math


def path_length(points):
    """Sum of straight segments between consecutive (x, y) samples."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def min_clearance(ranges, range_min, range_max):
    """Closest valid LaserScan return, or None if the scan had no valid returns.

    inf/nan and readings outside [range_min, range_max] are what the TB3 lidar
    reports for "nothing hit", so they are skipped rather than treated as 0.
    """
    valid = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    return min(valid) if valid else None


def time_in_state(transitions, state, end_time):
    """Seconds spent in `state`, given time-ordered (stamp, state) changes.

    Each state holds until the next transition; the last one holds until end_time.
    """
    held = 0.0
    for i, (stamp, current) in enumerate(transitions):
        until = transitions[i + 1][0] if i + 1 < len(transitions) else end_time
        if until < stamp:
            raise ValueError('transitions must be in time order and end_time after the last one')
        if current == state:
            held += until - stamp
    return held
