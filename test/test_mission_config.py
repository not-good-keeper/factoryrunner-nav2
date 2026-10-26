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


def test_obstacle_block_parsed():
    cfg = parse_mission(_doc(laps=3, obstacle_laps=[2],
                             obstacle={'leg': 'B', 'x': 2.5, 'y': -1, 'spawn_distance': 2}))
    assert cfg.obstacle.leg_station == 'B'
    assert (cfg.obstacle.x, cfg.obstacle.y, cfg.obstacle.spawn_distance) == (2.5, -1.0, 2.0)


def test_obstacle_defaults_spawn_distance():
    cfg = parse_mission(_doc(obstacle={'leg': 'A', 'x': 0, 'y': 0}))
    assert cfg.obstacle.spawn_distance == 3.0


def test_obstacle_laps_need_obstacle_block():
    with pytest.raises(MissionConfigError, match='mission.obstacle'):
        parse_mission(_doc(laps=2, obstacle_laps=[1]))


@pytest.mark.parametrize('obstacle, message', [
    ({'leg': 'C', 'x': 0, 'y': 0}, 'not a station'),
    ({'leg': 'A', 'x': 0}, 'missing'),
    ({'leg': 'A', 'x': 0, 'y': 0, 'spawn_distance': 0}, 'positive'),
    ('A', 'mapping'),
])
def test_bad_obstacle_block(obstacle, message):
    with pytest.raises(MissionConfigError, match=message):
        parse_mission(_doc(obstacle=obstacle))


def test_obstacle_motion_parsed():
    cfg = parse_mission(_doc(obstacle={'leg': 'B', 'x': 1, 'y': 2, 'motion':
                                       {'to_x': 3, 'to_y': 4, 'speed': 0.25}}))
    assert (cfg.obstacle.motion.to_x, cfg.obstacle.motion.to_y) == (3.0, 4.0)
    assert cfg.obstacle.motion.speed == 0.25


@pytest.mark.parametrize('motion', [{'to_x': 1, 'to_y': 1}, {'to_x': 1, 'to_y': 1, 'speed': 0}])
def test_bad_obstacle_motion(motion):
    with pytest.raises(MissionConfigError, match='motion'):
        parse_mission(_doc(obstacle={'leg': 'A', 'x': 0, 'y': 0, 'motion': motion}))
