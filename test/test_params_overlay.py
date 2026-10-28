from pathlib import Path

from factoryrunner.params_overlay import merge, write_merged
import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]


def test_merge_is_recursive_and_replaces_leaves_and_lists():
    base = {'a': {'b': 1, 'c': [1, 2]}, 'd': 'keep'}
    merged = merge(base, {'a': {'c': [3], 'e': 4}})
    assert merged == {'a': {'b': 1, 'c': [3], 'e': 4}, 'd': 'keep'}
    assert base['a']['c'] == [1, 2]


def test_overlay_must_be_a_mapping(tmp_path):
    base = tmp_path / 'base.yaml'
    base.write_text('a: 1\n')
    bad = tmp_path / 'bad.yaml'
    bad.write_text('- just a list\n')
    with pytest.raises(ValueError):
        write_merged(base, [bad], out_dir=tmp_path)


@pytest.mark.parametrize('overlay', sorted((REPO / 'config' / 'overlays').glob('*.yaml')))
def test_repo_overlays_only_touch_existing_nodes(overlay, tmp_path):
    base = yaml.safe_load((REPO / 'config' / 'nav2_params.yaml').read_text())
    for node in yaml.safe_load(overlay.read_text()):
        # slam_toolbox only runs in SLAM mode, so it has no section in the base file.
        assert node in base or node == 'slam_toolbox', f'{overlay.name} adds unknown node {node}'
    merged = yaml.safe_load(Path(write_merged(REPO / 'config' / 'nav2_params.yaml', [overlay],
                                              out_dir=tmp_path)).read_text())
    assert merged['bt_navigator'] == base['bt_navigator']


def test_velocity_stop_overlay_keeps_slowdown_and_approach():
    overlay = yaml.safe_load((REPO / 'config' / 'overlays' / 'velocity_stop.yaml').read_text())
    polygons = overlay['collision_monitor']['ros__parameters']['polygons']
    assert 'StopZone' not in polygons
    assert {'SlowdownZone', 'FootprintApproach'} <= set(polygons)
