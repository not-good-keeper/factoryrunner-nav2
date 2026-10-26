import math

from factoryrunner.crossing import (crossing_positions, CrossingMover, UPDATE_HZ,
                                    wall_seconds_for)
import pytest


def test_crossing_runs_start_to_end_at_speed():
    pts = crossing_positions((0.0, 0.0), (2.0, 0.0), speed=0.5)
    assert pts[0] == (0.0, 0.0)
    assert pts[-1] == pytest.approx((2.0, 0.0))
    assert wall_seconds_for(pts) == pytest.approx(4.0)
    steps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
    assert max(steps) == pytest.approx(0.5 / UPDATE_HZ)


def test_crossing_with_no_distance_stays_put():
    assert crossing_positions((1.0, 1.0), (1.0, 1.0), speed=0.3) == [(1.0, 1.0), (1.0, 1.0)]


def test_crossing_rejects_zero_speed():
    with pytest.raises(ValueError):
        crossing_positions((0, 0), (1, 0), speed=0)


def test_stopped_mover_can_be_joined():
    # Regression: a flag named _stop shadowed Thread._stop and made join() raise.
    mover = CrossingMover('no_such_model', [(0.0, 0.0), (1.0, 0.0)])
    mover.stop()
    mover.start()
    mover.join(timeout=5.0)
    assert not mover.is_alive()
