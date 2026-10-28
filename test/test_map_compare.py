from factoryrunner.map_compare import agreement, classify, FREE, OCCUPIED, resample, UNKNOWN
import numpy as np
import pytest


def test_classify_matches_map_server_thresholds():
    cells = classify([[0, 254, 205]], occupied_thresh=0.65, free_thresh=0.25)
    assert cells.tolist() == [[OCCUPIED, FREE, UNKNOWN]]


def test_resample_same_grid_is_identity():
    cells = np.array([[OCCUPIED, FREE], [UNKNOWN, FREE]])
    out = resample(cells, (0.0, 0.0), 0.05, (2, 2), (0.0, 0.0), 0.05)
    assert out.tolist() == cells.tolist()


def test_resample_outside_the_source_is_unknown():
    cells = np.array([[OCCUPIED]])
    out = resample(cells, (0.0, 0.0), 0.05, (1, 2), (0.0, 0.0), 0.05)
    assert out.tolist() == [[OCCUPIED, UNKNOWN]]


def test_agreement_tolerates_small_offsets_and_ignores_unobserved_cells():
    reference = np.full((5, 5), FREE)
    reference[2, 2] = OCCUPIED
    reference[0, 0] = OCCUPIED  # never observed by the built map
    built = np.full((5, 5), FREE)
    built[0, 0] = UNKNOWN
    built[2, 3] = OCCUPIED  # one cell off
    result = agreement(built, reference, tolerance_cells=1)
    assert result['occupied_recall'] == pytest.approx(1.0)
    assert result['occupied_precision'] == pytest.approx(1.0)
    assert result['observed_cells'] == 24


def test_agreement_flags_phantom_obstacles():
    reference = np.full((4, 4), FREE)
    built = np.full((4, 4), FREE)
    built[1, 1] = OCCUPIED
    result = agreement(built, reference)
    assert result['occupied_precision'] == pytest.approx(0.0)
    assert result['occupied_recall'] is None
