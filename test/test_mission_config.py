from pathlib import Path

from factoryrunner.mission_config import load_mission, MissionConfigError, parse_mission
import pytest

REPO_MISSION = Path(__file__).resolve().parents[1] / 'config' / 'mission.yaml'


def _doc(**overrides):
    mission = {
        'job_id': 'JOB-TEST',
        'initial_pose': {'x': 0.0, 'y': 0.0},
        'stations': [
            {'name': 'A', 'x': 1.0, 'y': 2.0, 'yaw': 1.57},
            {'name': 'B', 'x': 3, 'y': 4},
        ],
    }
    mission.update(overrides)
    return {'mission': mission}


def test_repo_mission_file_loads():
    cfg = load_mission(REPO_MISSION)
    assert cfg.stations
    assert all(lap <= cfg.laps for lap in cfg.obstacle_laps)


def test_defaults_and_int_coordinates():
    cfg = parse_mission(_doc())
    assert cfg.frame_id == 'map'
    assert cfg.laps == 1
    assert cfg.stations[1].pose.x == 3.0
    assert cfg.stations[1].pose.yaw == 0.0


@pytest.mark.parametrize('stations', [None, [], 'A'])
def test_rejects_missing_or_empty_stations(stations):
    with pytest.raises(MissionConfigError):
        parse_mission(_doc(stations=stations))


def test_rejects_duplicate_station_names():
    with pytest.raises(MissionConfigError, match='duplicate'):
        parse_mission(_doc(stations=[{'name': 'A', 'x': 0, 'y': 0}] * 2))


def test_rejects_boolean_coordinate():
    with pytest.raises(MissionConfigError, match='number'):
        parse_mission(_doc(stations=[{'name': 'A', 'x': True, 'y': 0}]))


def test_rejects_obstacle_lap_out_of_range():
    with pytest.raises(MissionConfigError, match='outside'):
        parse_mission(_doc(laps=3, obstacle_laps=[4]))


def test_missing_initial_pose():
    doc = _doc()
    del doc['mission']['initial_pose']
    with pytest.raises(MissionConfigError, match='initial_pose'):
        parse_mission(doc)


def test_unreadable_file(tmp_path):
    with pytest.raises(MissionConfigError, match='cannot read'):
        load_mission(tmp_path / 'nope.yaml')


def test_bad_yaml(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('mission: [unclosed')
    with pytest.raises(MissionConfigError, match='not valid YAML'):
        load_mission(bad)
