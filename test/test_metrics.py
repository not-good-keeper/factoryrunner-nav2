import math

from factoryrunner.metrics import (in_speed_zone, leg_metrics, min_clearance, MONITOR_SLOWDOWN,
                                   MONITOR_STOP, path_length, time_in_state)
import pytest


def test_path_length_right_angle():
    assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)


@pytest.mark.parametrize('points', [[], [(1.0, 1.0)]])
def test_path_length_too_few_points(points):
    assert path_length(points) == 0.0


def test_min_clearance_skips_invalid_returns():
    ranges = [math.inf, 0.0, 1.2, math.nan, 0.4, 25.0]
    assert min_clearance(ranges, range_min=0.12, range_max=20.0) == pytest.approx(0.4)


def test_min_clearance_all_invalid():
    assert min_clearance([math.inf, math.nan], 0.12, 20.0) is None


def test_time_in_state_last_state_runs_to_end():
    transitions = [(0.0, 'free'), (2.0, 'slowdown'), (5.0, 'free'), (6.0, 'slowdown')]
    assert time_in_state(transitions, 'slowdown', end_time=10.0) == pytest.approx(7.0)


def test_time_in_state_never_entered():
    assert time_in_state([(0.0, 'free')], 'stop', end_time=3.0) == 0.0


def test_time_in_state_rejects_unordered():
    with pytest.raises(ValueError):
        time_in_state([(5.0, 'free'), (1.0, 'stop')], 'stop', end_time=6.0)


@pytest.mark.parametrize('limit, inside', [(0.0, False), (40.0, True), (100.0, False)])
def test_in_speed_zone(limit, inside):
    assert in_speed_zone(limit) is inside


def test_leg_metrics_combines_samples():
    m = leg_metrics(
        start=10.0, end=30.0,
        positions=[(0, 0), (3, 0), (3, 4)],
        clearances=[1.5, None, 0.6, 0.9],
        monitor_transitions=[(10.0, 0), (12.0, MONITOR_SLOWDOWN), (15.0, MONITOR_STOP),
                             (16.0, MONITOR_SLOWDOWN), (17.0, 0)],
        zone_transitions=[(10.0, False), (20.0, True)],
    )
    assert m['duration_s'] == pytest.approx(20.0)
    assert m['path_length_m'] == pytest.approx(7.0)
    assert m['min_clearance_m'] == pytest.approx(0.6)
    assert m['slowdown_s'] == pytest.approx(4.0)
    assert m['stop_s'] == pytest.approx(1.0)
    assert m['speed_zone_s'] == pytest.approx(10.0)


def test_leg_metrics_without_valid_scans():
    m = leg_metrics(0.0, 5.0, [], [None], [(0.0, 0)], [(0.0, False)])
    assert m['min_clearance_m'] is None
    assert m['path_length_m'] == 0.0
