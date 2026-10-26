"""Health checks for the running stack, as plain functions so pytest covers them without ROS.

Levels follow diagnostic_msgs/DiagnosticStatus: OK, WARN, ERROR, STALE.
"""

OK, WARN, ERROR, STALE = 0, 1, 2, 3
LEVEL_NAMES = {OK: 'OK', WARN: 'WARN', ERROR: 'ERROR', STALE: 'STALE'}

# nav2_msgs/CollisionMonitorState action_type
_MONITOR_ACTIONS = {0: 'none', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


def rate_hz(stamps, now, window_s):
    """Messages per second over the last window_s seconds, from receive stamps."""
    recent = [t for t in stamps if now - window_s < t <= now]
    return len(recent) / window_s


def scan_level(hz, expected_hz):
    """Return (level, message) for the LiDAR rate; the waffle's sim LiDAR runs at 5 Hz."""
    if hz == 0:
        return ERROR, 'no scans'
    if hz < 0.6 * expected_hz:
        return WARN, f'{hz:.1f} Hz, expected about {expected_hz:.0f}'
    return OK, f'{hz:.1f} Hz'


def tf_level(age_s, warn_s, error_s):
    """Return (level, message) for the age of the newest map->odom transform.

    AMCL republishes map->odom on every processed scan, so a transform older than a
    couple of scan periods means localization, or the process hosting it, has stopped.
    """
    if age_s is None:
        return ERROR, 'map->odom unavailable'
    if age_s >= error_s:
        return ERROR, f'map->odom is {age_s:.1f} s old'
    if age_s >= warn_s:
        return WARN, f'map->odom is {age_s:.1f} s old'
    return OK, f'map->odom {age_s:.2f} s old'


def lifecycle_level(states):
    """Return (level, message) from {node: state label, or None if it did not answer}."""
    silent = sorted(n for n, s in states.items() if s is None)
    inactive = sorted(f'{n}={s}' for n, s in states.items() if s is not None and s != 'active')
    if silent:
        return ERROR, 'no answer from ' + ', '.join(silent)
    if inactive:
        return WARN, 'not active: ' + ', '.join(inactive)
    return OK, f'{len(states)} nodes active'


def monitor_level(action_type):
    """Return (level, message) for the collision monitor's current action."""
    if action_type is None:
        return OK, 'no state published yet (publishes on change only)'
    name = _MONITOR_ACTIONS.get(action_type, f'action {action_type}')
    if action_type == 1:
        return WARN, 'stop field active'
    return OK, name


def worst(levels):
    return max(levels, default=OK)
