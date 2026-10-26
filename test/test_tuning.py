from factoryrunner.tuning import by_node, defaults, objective, SEARCH_SPACE
import pytest


def _leg(duration=20.0, result='SUCCEEDED', recoveries=0, clearance=0.8, stops=0):
    return {'duration_s': duration, 'result': result, 'recoveries': recoveries,
            'min_clearance_m': clearance, 'stop_events': stops}


def test_clean_lap_scores_its_lap_time():
    assert objective([_leg(18), _leg(30), _leg(21), _leg(22)], 4) == pytest.approx(91.0)


def test_penalties_add_up():
    legs = [_leg(20, recoveries=1), _leg(30, result='FAILED', clearance=0.10, stops=2)]
    # 50 s + 60 failed + 10 recovery + 20 * (0.15 / 0.25) clearance + 5 * 2 stops
    assert objective(legs, 2) == pytest.approx(50 + 60 + 10 + 12 + 10)


def test_unrun_legs_count_as_failed():
    assert objective([_leg(20)], 4) == pytest.approx(20 + 3 * 60)


def test_missing_clearance_is_not_penalised():
    assert objective([_leg(20, clearance=None)], 1) == pytest.approx(20)


def test_defaults_lie_inside_the_search_space():
    for p in SEARCH_SPACE:
        assert p.low <= p.default <= p.high


def test_by_node_groups_parameters():
    grouped = by_node(defaults())
    assert sorted(grouped) == ['controller_server', 'local_costmap/local_costmap']
    assert ('FollowPath.vx_max', 0.5) in grouped['controller_server']


def test_defaults_match_nav2_params():
    from pathlib import Path

    import yaml

    params = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config' /
                             'nav2_params.yaml').read_text())
    for p in SEARCH_SPACE:
        node = params
        for key in p.node.split('/'):
            node = node[key]
        value = node['ros__parameters']
        for key in p.name.split('.'):
            value = value[key]
        assert value == pytest.approx(p.default), p.name
