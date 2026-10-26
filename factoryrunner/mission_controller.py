"""Drive the robot through the station list in mission.yaml and log each leg.

ros2 run factoryrunner mission_controller
ros2 run factoryrunner mission_controller --ros-args -p mission_file:=/path/to/mission.yaml
"""

import csv
from datetime import datetime
import math
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from factoryrunner.mission_config import load_mission, MissionConfigError
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter

CSV_FIELDS = ['run_id', 'job_id', 'leg', 'station', 'result',
              'start_s', 'end_s', 'duration_s', 'recoveries']


def to_pose_stamped(navigator, pose, frame_id):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.position.x = pose.x
    msg.pose.position.y = pose.y
    # planar robot: only yaw matters
    msg.pose.orientation.z = math.sin(pose.yaw / 2.0)
    msg.pose.orientation.w = math.cos(pose.yaw / 2.0)
    return msg


def sim_seconds(navigator):
    return navigator.get_clock().now().nanoseconds / 1e9


def append_rows(csv_path, rows):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not csv_path.exists()
    with csv_path.open('a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerows(rows)


def run_leg(navigator, mission, leg, station):
    total = len(mission.stations)
    goal = to_pose_stamped(navigator, station.pose, mission.frame_id)
    start = sim_seconds(navigator)

    if not navigator.goToPose(goal):
        print(f'[{leg}/{total}] {station.name}: goal REJECTED')
        return TaskResult.FAILED, start, sim_seconds(navigator), 0

    print(f'[{leg}/{total}] Goal accepted: {station.name} '
          f'({station.pose.x:.2f}, {station.pose.y:.2f})')
    recoveries = 0
    last_print = start
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is None:
            continue
        recoveries = feedback.number_of_recoveries
        now = sim_seconds(navigator)
        if now - last_print >= 1.0:
            print(f'      {feedback.distance_remaining:5.2f} m to go, recoveries {recoveries}')
            last_print = now

    result = navigator.getResult()
    return result, start, sim_seconds(navigator), recoveries


def main(argv=None):
    rclpy.init(args=argv if argv is not None else sys.argv)
    navigator = BasicNavigator('factoryrunner_mission')
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    default_mission = Path(get_package_share_directory('factoryrunner')) / 'config' / 'mission.yaml'
    mission_file = navigator.declare_parameter('mission_file', str(default_mission)).value
    results_dir = navigator.declare_parameter('results_dir', str(Path.cwd() / 'results')).value

    try:
        mission = load_mission(mission_file)
    except MissionConfigError as exc:
        print(f'mission file problem: {exc}', file=sys.stderr)
        navigator.destroy_node()
        rclpy.shutdown()
        return 2

    run_id = datetime.now().strftime('%Y%m%d-%H%M%S')
    print(f'FACTORYRUNNER | {mission.job_id} | run {run_id}')

    navigator.setInitialPose(to_pose_stamped(navigator, mission.initial_pose, mission.frame_id))
    navigator.waitUntilNav2Active()

    rows = []
    succeeded = 0
    mission_start = sim_seconds(navigator)
    for leg, station in enumerate(mission.stations, start=1):
        result, start, end, recoveries = run_leg(navigator, mission, leg, station)
        rows.append({
            'run_id': run_id, 'job_id': mission.job_id, 'leg': leg, 'station': station.name,
            'result': result.name, 'start_s': f'{start:.2f}', 'end_s': f'{end:.2f}',
            'duration_s': f'{end - start:.2f}', 'recoveries': recoveries,
        })
        print(f'[{leg}/{len(mission.stations)}] {station.name}: {result.name} '
              f'in {end - start:.1f} s')
        if result != TaskResult.SUCCEEDED:
            break
        succeeded += 1

    elapsed = sim_seconds(navigator) - mission_start
    csv_path = Path(results_dir) / 'missions.csv'
    append_rows(csv_path, rows)

    complete = succeeded == len(mission.stations)
    print()
    print('MISSION COMPLETE' if complete else 'MISSION FAILED')
    print(f'Goals succeeded: {succeeded}/{len(mission.stations)}')
    print(f'Elapsed sim time: {elapsed:.1f} s')
    print(f'Logged to {csv_path}')

    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if complete else 1


if __name__ == '__main__':
    sys.exit(main())
