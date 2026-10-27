"""Drive the robot through the station list in mission.yaml and log each leg.

ros2 run factoryrunner mission_controller
ros2 run factoryrunner mission_controller --ros-args -p mission_file:=/path/to/mission.yaml
ros2 run factoryrunner mission_controller --ros-args -p leg_timeout_s:=90.0

A leg that has no result after leg_timeout_s sim seconds (default 120) is canceled and
logged as CANCELED, and the mission stops there.
"""

from datetime import datetime
import math
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from factoryrunner.mission_config import load_mission, MissionConfigError
from factoryrunner.mission_log import (append_row, check_leg_timeout, default_results_dir,
                                       leg_row, leg_timed_out)
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter

CANCEL_WAIT_S = 10.0


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


def warn_unless_cyclone(navigator):
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        # Nav2's container froze under the default Fast DDS here; see docs/CHALLENGES.md.
        navigator.get_logger().warn(
            'RMW_IMPLEMENTATION is not rmw_cyclonedds_cpp; '
            'run: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')


def cancel_leg(navigator):
    """Cancel the active goal, giving up after CANCEL_WAIT_S.

    BasicNavigator.cancelTask() waits forever for the cancel response, and a frozen Nav2
    never sends one.
    """
    if navigator.goal_handle is None:
        return False
    future = navigator.goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=CANCEL_WAIT_S)
    return future.done()


def run_leg(navigator, mission, leg, station, leg_timeout_s, on_tick=None):
    """Drive one leg and return (result, start_s, end_s, recoveries).

    on_tick, if given, is called on every pass of the wait loop; the trial runner uses it to
    drop the obstacle in when the robot gets close.
    """
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
        if leg_timed_out(start, sim_seconds(navigator), leg_timeout_s):
            print(f'[{leg}/{total}] {station.name}: no result after '
                  f'{leg_timeout_s:.0f} s, canceling')
            if not cancel_leg(navigator):
                print(f'      cancel not acknowledged within {CANCEL_WAIT_S:.0f} s; '
                      'Nav2 may be frozen')
            return TaskResult.CANCELED, start, sim_seconds(navigator), recoveries
        if on_tick is not None:
            on_tick()
        feedback = navigator.getFeedback()
        if feedback is None:
            continue
        recoveries = feedback.number_of_recoveries
        now = sim_seconds(navigator)
        if now - last_print >= 1.0:
            print(f'      {feedback.distance_remaining:5.2f} m to go, recoveries {recoveries}')
            last_print = now

    return navigator.getResult(), start, sim_seconds(navigator), recoveries


def main(argv=None):
    rclpy.init(args=argv if argv is not None else sys.argv)
    navigator = BasicNavigator('factoryrunner_mission')
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
        print(f'cannot start mission: {exc}', file=sys.stderr)
        navigator.destroy_node()
        rclpy.shutdown()
        return 2

    run_id = datetime.now().strftime('%Y%m%d-%H%M%S')
    print(f'FACTORYRUNNER | {mission.job_id} | run {run_id}')

    navigator.setInitialPose(to_pose_stamped(navigator, mission.initial_pose, mission.frame_id))
    navigator.waitUntilNav2Active()

    csv_path = Path(results_dir) / 'missions.csv'
    succeeded = 0
    mission_start = sim_seconds(navigator)
    for leg, station in enumerate(mission.stations, start=1):
        result, start, end, recoveries = run_leg(navigator, mission, leg, station, leg_timeout_s)
        append_row(csv_path, leg_row(run_id, mission.job_id, leg, station.name, result.name,
                                     start, end, recoveries))
        print(f'[{leg}/{len(mission.stations)}] {station.name}: {result.name} '
              f'in {end - start:.1f} s')
        if result != TaskResult.SUCCEEDED:
            break
        succeeded += 1

    elapsed = sim_seconds(navigator) - mission_start

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
