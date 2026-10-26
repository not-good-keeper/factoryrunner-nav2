from factoryrunner.health import (ERROR, lifecycle_level, monitor_level, OK, rate_hz,
                                  scan_level, tf_level, WARN, worst)
import pytest


def test_rate_counts_only_the_window():
    stamps = [0.5, 1.0, 9.2, 9.4, 9.6, 9.8, 10.0]
    assert rate_hz(stamps, now=10.0, window_s=1.0) == pytest.approx(5.0)


def test_rate_with_no_messages():
    assert rate_hz([], now=10.0, window_s=2.0) == 0.0


@pytest.mark.parametrize('hz, level', [(0.0, ERROR), (2.0, WARN), (4.9, OK)])
def test_scan_level(hz, level):
    assert scan_level(hz, expected_hz=5.0)[0] == level


@pytest.mark.parametrize('age, level', [(None, ERROR), (0.3, OK), (1.2, WARN), (3.0, ERROR)])
def test_tf_level(age, level):
    assert tf_level(age, warn_s=1.0, error_s=2.5)[0] == level


def test_lifecycle_silent_node_is_an_error():
    level, msg = lifecycle_level({'amcl': 'active', 'bt_navigator': None})
    assert level == ERROR
    assert 'bt_navigator' in msg


def test_lifecycle_inactive_node_is_a_warning():
    level, msg = lifecycle_level({'amcl': 'active', 'controller_server': 'inactive'})
    assert level == WARN
    assert 'controller_server=inactive' in msg


def test_lifecycle_all_active():
    assert lifecycle_level({'amcl': 'active', 'planner_server': 'active'})[0] == OK


@pytest.mark.parametrize('action, level', [(None, OK), (0, OK), (2, OK), (1, WARN)])
def test_monitor_level(action, level):
    assert monitor_level(action)[0] == level


def test_worst():
    assert worst([OK, WARN, OK]) == WARN
    assert worst([]) == OK
