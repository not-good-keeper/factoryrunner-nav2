"""Load and validate mission.yaml. Kept free of ROS imports so pytest runs without a sim."""

from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml


class MissionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Station:
    name: str
    pose: Pose2D


@dataclass(frozen=True)
class ObstacleConfig:
    leg_station: str  # the box goes on the leg that ends at this station
    x: float
    y: float
    spawn_distance: float = 3.0  # spawn once the robot is this close to (x, y)


@dataclass(frozen=True)
class MissionConfig:
    job_id: str
    frame_id: str
    initial_pose: Pose2D
    stations: list
    laps: int = 1
    obstacle_laps: list = field(default_factory=list)
    obstacle: ObstacleConfig = None


def _number(raw, where):
    # bool is an int subclass; "x: yes" in YAML must not pass as 1.0
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise MissionConfigError(f'{where} must be a number, got {raw!r}')
    if not math.isfinite(raw):
        raise MissionConfigError(f'{where} must be finite, got {raw!r}')
    return float(raw)


def _pose(raw, where):
    if not isinstance(raw, dict):
        raise MissionConfigError(f'{where} must be a mapping with x, y and optional yaw')
    missing = {'x', 'y'} - raw.keys()
    if missing:
        raise MissionConfigError(f'{where} is missing {sorted(missing)}')
    return Pose2D(
        x=_number(raw['x'], f'{where}.x'),
        y=_number(raw['y'], f'{where}.y'),
        yaw=_number(raw.get('yaw', 0.0), f'{where}.yaw'),
    )


def _obstacle(raw, station_names):
    where = 'mission.obstacle'
    if not isinstance(raw, dict):
        raise MissionConfigError(f'{where} must be a mapping with leg, x, y')
    missing = {'leg', 'x', 'y'} - raw.keys()
    if missing:
        raise MissionConfigError(f'{where} is missing {sorted(missing)}')
    if raw['leg'] not in station_names:
        raise MissionConfigError(f"{where}.leg {raw['leg']!r} is not a station name")
    spawn_distance = _number(raw.get('spawn_distance', 3.0), f'{where}.spawn_distance')
    if spawn_distance <= 0:
        raise MissionConfigError(f'{where}.spawn_distance must be positive, got {spawn_distance}')
    return ObstacleConfig(
        leg_station=raw['leg'],
        x=_number(raw['x'], f'{where}.x'),
        y=_number(raw['y'], f'{where}.y'),
        spawn_distance=spawn_distance,
    )


def parse_mission(doc):
    if not isinstance(doc, dict) or 'mission' not in doc:
        raise MissionConfigError("top-level key 'mission' not found")
    m = doc['mission']
    if not isinstance(m, dict):
        raise MissionConfigError("'mission' must be a mapping")

    raw_stations = m.get('stations')
    if not isinstance(raw_stations, list) or not raw_stations:
        raise MissionConfigError('mission.stations must be a non-empty list')

    stations = []
    seen = set()
    for i, raw in enumerate(raw_stations):
        where = f'mission.stations[{i}]'
        if not isinstance(raw, dict) or not isinstance(raw.get('name'), str) or not raw['name']:
            raise MissionConfigError(f"{where} needs a non-empty string 'name'")
        name = raw['name']
        if name in seen:
            raise MissionConfigError(f'duplicate station name {name!r}')
        seen.add(name)
        stations.append(Station(name=name, pose=_pose(raw, where)))

    laps = m.get('laps', 1)
    if isinstance(laps, bool) or not isinstance(laps, int) or laps < 1:
        raise MissionConfigError(f'mission.laps must be an integer >= 1, got {laps!r}')

    obstacle_laps = m.get('obstacle_laps', []) or []
    if not isinstance(obstacle_laps, list):
        raise MissionConfigError('mission.obstacle_laps must be a list')
    for lap in obstacle_laps:
        if isinstance(lap, bool) or not isinstance(lap, int) or not 1 <= lap <= laps:
            raise MissionConfigError(f'obstacle lap {lap!r} is outside 1..{laps}')

    obstacle = None
    if m.get('obstacle') is not None:
        obstacle = _obstacle(m['obstacle'], seen)
    elif obstacle_laps:
        raise MissionConfigError('mission.obstacle_laps is set but mission.obstacle is not')

    return MissionConfig(
        job_id=str(m.get('job_id', 'JOB-001')),
        frame_id=str(m.get('frame_id', 'map')),
        initial_pose=_pose(m.get('initial_pose'), 'mission.initial_pose'),
        stations=stations,
        laps=laps,
        obstacle_laps=sorted(set(obstacle_laps)),
        obstacle=obstacle,
    )


def load_mission(path):
    path = Path(path)
    try:
        text = path.read_text()
    except OSError as exc:
        raise MissionConfigError(f'cannot read {path}: {exc}') from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise MissionConfigError(f'{path} is not valid YAML: {exc}') from exc
    return parse_mission(doc)
