import csv
import math

from factoryrunner.mission_log import (append_row, check_leg_timeout, CSV_FIELDS, leg_row,
                                       leg_timed_out)
import pytest


def _row(leg, result='SUCCEEDED'):
    return leg_row('run-1', 'JOB-001', leg, f'S{leg}', result, 10.0, 25.5, 2)


def _read(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def test_leg_row_formats_times():
    row = _row(3)
    assert row['start_s'] == '10.00'
    assert row['end_s'] == '25.50'
    assert row['duration_s'] == '15.50'
    assert set(row) == set(CSV_FIELDS)


def test_append_row_writes_header_once(tmp_path):
    path = tmp_path / 'results' / 'missions.csv'
    append_row(path, _row(1))
    append_row(path, _row(2, 'CANCELED'))
    lines = path.read_text().splitlines()
    assert lines[0] == ','.join(CSV_FIELDS)
    assert len(lines) == 3
    assert [r['result'] for r in _read(path)] == ['SUCCEEDED', 'CANCELED']


def test_append_row_adds_header_to_empty_file(tmp_path):
    path = tmp_path / 'missions.csv'
    path.touch()
    append_row(path, _row(1))
    assert _read(path)[0]['leg'] == '1'


def test_leg_timed_out_at_limit():
    assert not leg_timed_out(100.0, 219.9, 120.0)
    assert leg_timed_out(100.0, 220.0, 120.0)


def test_leg_timed_out_ignores_clock_going_backwards():
    assert not leg_timed_out(100.0, 5.0, 120.0)


def test_check_leg_timeout_accepts_int():
    assert check_leg_timeout(90) == 90.0


@pytest.mark.parametrize('bad', [0, -5.0, math.inf, math.nan, True, '120'])
def test_check_leg_timeout_rejects(bad):
    with pytest.raises(ValueError):
        check_leg_timeout(bad)
