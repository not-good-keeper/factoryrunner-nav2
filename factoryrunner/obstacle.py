"""Drop the pallet_box model into the running depot world, or take it out again.

ros2 run factoryrunner obstacle spawn --x 8.0 --y -1.0     (map-frame metres)
ros2 run factoryrunner obstacle remove

Gazebo answers create and remove as soon as the request is queued: a duplicate name or a
missing entity still gets a success reply, and the real error only shows up in the Gazebo
log. The headless server has no scene broadcaster to query, so check the effect in /scan or
the costmaps when it matters.
"""

import argparse
from pathlib import Path
import subprocess
import sys

WORLD = 'depot'
# sim.launch.py spawns the robot at Gazebo (-8, 0) and AMCL starts it at map (0, 0). The depot
# map is not rotated against the world, so the frames differ by this translation only.
MAP_ORIGIN_IN_GAZEBO = (-8.0, 0.0)
DEFAULT_NAME = 'pallet_box'
SPAWN_Z = 0.25  # half the box height, so it sits on the floor
COMMAND_TIMEOUT_S = 30


class ObstacleError(RuntimeError):
    pass


def map_to_gazebo(x, y):
    return x + MAP_ORIGIN_IN_GAZEBO[0], y + MAP_ORIGIN_IN_GAZEBO[1]


def spawn_command(model_file, name, x, y, world=WORLD):
    gx, gy = map_to_gazebo(x, y)
    return ['ros2', 'run', 'ros_gz_sim', 'create', '-world', world, '-file', str(model_file),
            '-name', name, '-x', f'{gx:.3f}', '-y', f'{gy:.3f}', '-z', f'{SPAWN_Z}']


def remove_command(name, world=WORLD):
    return ['gz', 'service', '-s', f'/world/{world}/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', f'name: "{name}" type: MODEL']


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=COMMAND_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ObstacleError(f'{cmd[0]} failed: {exc}') from exc


def spawn(name, x, y):
    from ament_index_python.packages import get_package_share_directory

    model = Path(get_package_share_directory('factoryrunner')) / 'models' / 'pallet_box.sdf'
    proc = _run(spawn_command(model, name, x, y))
    if proc.returncode != 0:
        raise ObstacleError(f'spawning {name} failed: {(proc.stderr or proc.stdout).strip()}')


def remove(name):
    proc = _run(remove_command(name))
    # 'data: true' only means the request was accepted; see the module docstring.
    if proc.returncode != 0 or 'data: true' not in proc.stdout:
        raise ObstacleError(f'removing {name} failed: {(proc.stderr or proc.stdout).strip()}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='action', required=True)
    add = sub.add_parser('spawn', help='spawn the box at a map-frame position')
    add.add_argument('--x', type=float, required=True)
    add.add_argument('--y', type=float, required=True)
    add.add_argument('--name', default=DEFAULT_NAME)
    rm = sub.add_parser('remove', help='remove a spawned box')
    rm.add_argument('--name', default=DEFAULT_NAME)
    args = parser.parse_args(argv)

    try:
        if args.action == 'spawn':
            spawn(args.name, args.x, args.y)
            print(f'requested {args.name} at map ({args.x:.2f}, {args.y:.2f})')
        else:
            remove(args.name)
            print(f'requested removal of {args.name}')
    except ObstacleError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
