"""Leg timeouts and the per-leg CSV log. No ROS imports, so pytest covers it without a sim."""

import csv
import math
from pathlib import Path

CSV_FIELDS = ['run_id', 'job_id', 'leg', 'station', 'result',
              'start_s', 'end_s', 'duration_s', 'recoveries']


def default_results_dir(share_dir):
    """Return results/ in the source checkout, so runs land in the repo wherever they start.

    With colcon's --symlink-install (what the README uses) the installed config/ links back
    into the checkout. A plain install has no checkout to find, so fall back to ./results.
    """
    repo = (Path(share_dir) / 'config').resolve().parent
    if (repo / 'package.xml').is_file() and (repo / '.git').exists():
        return repo / 'results'
    return Path.cwd() / 'results'


def check_leg_timeout(value):
    """Return the leg timeout in seconds, or raise ValueError if it can never fire sensibly."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'leg_timeout_s must be a number, got {value!r}')
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'leg_timeout_s must be a positive number of seconds, got {value!r}')
    return float(value)


def leg_timed_out(start_s, now_s, limit_s):
    # A sim clock that jumps backwards (Gazebo reset) gives a negative elapsed time,
    # which reads as "not yet" rather than as a timeout.
    return now_s - start_s >= limit_s


def leg_row(run_id, job_id, leg, station, result, start_s, end_s, recoveries):
    return {
        'run_id': run_id, 'job_id': job_id, 'leg': leg, 'station': station,
        'result': result, 'start_s': f'{start_s:.2f}', 'end_s': f'{end_s:.2f}',
        'duration_s': f'{end_s - start_s:.2f}', 'recoveries': recoveries,
    }


def append_row(csv_path, row, fields=CSV_FIELDS):
    """Append one leg, writing the header first if the file is new or empty.

    Called after every leg so a run that is killed mid-mission keeps the legs it finished.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not csv_path.exists() or csv_path.stat().st_size == 0
    with csv_path.open('a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)
