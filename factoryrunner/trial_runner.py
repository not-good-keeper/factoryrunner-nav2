"""Run the station loop for several laps, drop the pallet in on some of them, log every leg.

ros2 run factoryrunner trial_runner
ros2 run factoryrunner trial_runner --ros-args -p mission_file:=/path/to/mission.yaml

Laps, obstacle laps and the obstacle spot come from the mission file. Each leg becomes one row
of results/trials_<timestamp>.csv, written as soon as the leg ends. A failed leg is logged and
the run carries on; a leg that hits leg_timeout_s ends the run, since Nav2 may be stuck.
"""

from datetime import datetime
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from factoryrunner import obstacle
from factoryrunner.crossing import crossing_positions, CrossingMover
from factoryrunner.metrics import in_speed_zone, leg_metrics, min_clearance
from factoryrunner.mission_config import load_mission, MissionConfigError
from factoryrunner.mission_controller import (run_leg, sim_seconds, to_pose_stamped,
                                              warn_unless_cyclone)
from factoryrunner.mission_log import append_row, check_leg_timeout, default_results_dir
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TRIAL_FIELDS = ['run_id', 'lap', 'leg', 'station', 'obstacle', 'result', 'start_s', 'end_s',
                'duration_s', 'path_length_m', 'recoveries', 'min_clearance_m', 'slowdown_s',
                'stop_s', 'stop_events', 'speed_zone_s', 'obstacle_trigger_dist_m',
                'obstacle_spawn_wall_s']


def _fmt(value):
    return '' if value is None else f'{value:.2f}'


class LegRecorder:
    """Collects the samples one leg's metrics are computed from.

    The subscriptions live on the navigator node, so they run whenever BasicNavigator spins
    while it waits for a result.
    """

    def __init__(self, node):
        self._node = node
        self.position = None
        # The collision monitor and the speed filter only publish changes, so the state at the
        # end of one leg is the state at the start of the next.
        self._monitor_state = 0
        self._in_zone = False
        self.start(0.0)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        node.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        node.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self._on_monitor, 10)
        node.create_subscription(SpeedLimit, '/speed_limit', self._on_speed_limit, 10)

    def start(self, stamp):
        self.positions = [] if self.position is None else [self.position]
        self.clearances = []
        self.monitor_transitions = [(stamp, self._monitor_state)]
        self.zone_transitions = [(stamp, self._in_zone)]

    def metrics(self, start, end):
        return leg_metrics(start, end, self.positions, self.clearances,
                           self.monitor_transitions, self.zone_transitions)

    def _on_pose(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.positions.append(self.position)

    def _on_scan(self, msg):
        self.clearances.append(min_clearance(msg.ranges, msg.range_min, msg.range_max))

    def _on_monitor(self, msg):
        if msg.action_type != self._monitor_state:
            self._monitor_state = msg.action_type
            self.monitor_transitions.append((sim_seconds(self._node), msg.action_type))

    def _on_speed_limit(self, msg):
        inside = bool(msg.percentage) and in_speed_zone(msg.speed_limit)
        if inside != self._in_zone:
            self._in_zone = inside
            self.zone_transitions.append((sim_seconds(self._node), inside))


class ObstacleDrop:
    """on_tick hook that spawns the pallet once the robot is within spawn_distance of its spot."""

    def __init__(self, recorder, cfg, name):
        self._recorder = recorder
        self._spot = (cfg.x, cfg.y)
        self._spawn_distance = cfg.spawn_distance
        self._motion = cfg.motion
        self._mover = None
        self.name = name
        self.spawned = False
        self.failed = False
        self.trigger_dist = None
        self.spawn_wall_s = None

    def __call__(self):
        if self.spawned or self.failed or self._recorder.position is None:
            return
        dist = math.dist(self._recorder.position, self._spot)
        if dist > self._spawn_distance:
            return
        # The spawn command blocks for a second or two while the robot keeps driving, so log
        # both the trigger distance and how long the spawn took. Wall time, because the node's
        # sim clock does not advance while nothing spins it.
        self.trigger_dist = dist
        before = time.monotonic()
        try:
            obstacle.spawn(self.name, *self._spot)
        except obstacle.ObstacleError as exc:
            self.failed = True
            print(f'      obstacle spawn failed: {exc}')
            return
        self.spawned = True
        self.spawn_wall_s = time.monotonic() - before
        if self._motion is not None:
            path = crossing_positions(self._spot, (self._motion.to_x, self._motion.to_y),
                                      self._motion.speed)
            self._mover = CrossingMover(self.name, path)
            self._mover.start()
        print(f'      dropped {self.name} at ({self._spot[0]:.2f}, {self._spot[1]:.2f}), '
              f'robot {dist:.2f} m away')

    def remove(self):
        if self._mover is not None:
            self._mover.stop()
            self._mover.join(timeout=2.0)
            self._mover = None
        if not self.spawned:
            return
        try:
            obstacle.remove(self.name)
        except obstacle.ObstacleError as exc:
            print(f'      obstacle removal failed: {exc}')
        self.spawned = False


def run_trials(navigator, mission, csv_path, run_id, leg_timeout_s):
    """Drive mission.laps laps; return one dict of metrics per leg that ran."""
    recorder = LegRecorder(navigator)
    legs = []
    for lap in range(1, mission.laps + 1):
        obstacle_lap = lap in mission.obstacle_laps
        print(f'=== lap {lap}/{mission.laps}{" (obstacle)" if obstacle_lap else ""}')
        for leg, station in enumerate(mission.stations, start=1):
            drop = None
            if obstacle_lap and station.name == mission.obstacle.leg_station:
                drop = ObstacleDrop(recorder, mission.obstacle, f'pallet_lap{lap}')
            recorder.start(sim_seconds(navigator))
            try:
                result, start, end, recoveries = run_leg(navigator, mission, leg, station,
                                                         leg_timeout_s, on_tick=drop)
            finally:
                if drop is not None:
                    drop.remove()
            m = recorder.metrics(start, end)
            append_row(csv_path, {
                'run_id': run_id, 'lap': lap, 'leg': leg, 'station': station.name,
                'obstacle': 'failed' if drop and drop.failed else int(drop is not None),
                'result': result.name, 'start_s': _fmt(start), 'end_s': _fmt(end),
                'duration_s': _fmt(m['duration_s']), 'path_length_m': _fmt(m['path_length_m']),
                'recoveries': recoveries, 'min_clearance_m': _fmt(m['min_clearance_m']),
                'slowdown_s': _fmt(m['slowdown_s']), 'stop_s': _fmt(m['stop_s']),
                'stop_events': m['stop_events'],
                'speed_zone_s': _fmt(m['speed_zone_s']),
                'obstacle_trigger_dist_m': _fmt(drop.trigger_dist if drop else None),
                'obstacle_spawn_wall_s': _fmt(drop.spawn_wall_s if drop else None),
            }, TRIAL_FIELDS)
            legs.append(dict(m, lap=lap, station=station.name, result=result.name,
                             recoveries=recoveries))
            print(f'[lap {lap} leg {leg}] {station.name}: {result.name} in {end - start:.1f} s')
            if result == TaskResult.CANCELED:
                print('leg timed out; stopping the trials in case Nav2 is stuck')
                return legs
    return legs


def main(argv=None):
    rclpy.init(args=argv if argv is not None else sys.argv)
    navigator = BasicNavigator('factoryrunner_trials')
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    warn_unless_cyclone(navigator)

    share = Path(get_package_share_directory('factoryrunner'))
    default_mission = share / 'config' / 'mission.yaml'
    mission_file = navigator.declare_parameter('mission_file', str(default_mission)).value
    results_dir = navigator.declare_parameter(
        'results_dir', str(default_results_dir(share))).value
    leg_timeout_s = navigator.declare_parameter('leg_timeout_s', 120.0).value

    try:
        leg_timeout_s = check_leg_timeout(leg_timeout_s)
        mission = load_mission(mission_file)
    except (MissionConfigError, ValueError) as exc:
        print(f'cannot start trials: {exc}', file=sys.stderr)
        navigator.destroy_node()
        rclpy.shutdown()
        return 2

    run_id = datetime.now().strftime('%Y%m%d-%H%M%S')
    csv_path = Path(results_dir) / f'trials_{run_id}.csv'
    print(f'FACTORYRUNNER TRIALS | {mission.laps} laps, obstacle on {mission.obstacle_laps} '
          f'| run {run_id}')

    navigator.setInitialPose(to_pose_stamped(navigator, mission.initial_pose, mission.frame_id))
    navigator.waitUntilNav2Active()

    total = mission.laps * len(mission.stations)
    try:
        legs = run_trials(navigator, mission, csv_path, run_id, leg_timeout_s)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Every finished leg is already in the CSV; the interrupted one is not.
        print(f'\ninterrupted; legs finished so far are in {csv_path}')
        return 130
    print()
    succeeded = sum(leg['result'] == 'SUCCEEDED' for leg in legs)
    print(f'Legs succeeded: {succeeded}/{len(legs)} run of {total} planned')
    print(f'Logged to {csv_path}')

    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if succeeded == total else 1


if __name__ == '__main__':
    sys.exit(main())
