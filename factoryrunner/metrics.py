"""Per-leg trial metrics. No ROS imports: the trial runner feeds plain numbers in."""

import math

# nav2_msgs/CollisionMonitorState action_type values
MONITOR_STOP = 1
MONITOR_SLOWDOWN = 2


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


def in_speed_zone(speed_limit):
    """Return True if a percentage /speed_limit value means the robot is being slowed.

    The speed filter publishes 0 for "no limit" and 100 means full speed, so only values in
    between count as being in a zone.
    """
    return 0.0 < speed_limit < 100.0


def leg_metrics(start, end, positions, clearances, monitor_transitions, zone_transitions):
    """Compute the per-leg numbers for the trials CSV.

    positions are time-ordered (x, y) samples, clearances the per-scan minimum ranges (None for
    a scan with no valid return), monitor_transitions (stamp, action_type) changes and
    zone_transitions (stamp, in_zone) changes, each starting with the state at `start`.
    """
    valid = [c for c in clearances if c is not None]
    return {
        'duration_s': end - start,
        'path_length_m': path_length(positions),
        'min_clearance_m': min(valid) if valid else None,
        'slowdown_s': time_in_state(monitor_transitions, MONITOR_SLOWDOWN, end),
        'stop_s': time_in_state(monitor_transitions, MONITOR_STOP, end),
        'speed_zone_s': time_in_state(zone_transitions, True, end),
    }
