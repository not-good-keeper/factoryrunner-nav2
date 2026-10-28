#!/usr/bin/env python3
"""Drive the station loop with SLAM running and save the map it builds.

Start the sim in SLAM mode first, then run this from a sourced terminal within about a minute:
    ros2 launch factoryrunner sim.launch.py slam:=True headless:=True use_rviz:=False
    python3 scripts/map_depot.py --out maps/depot_slam

slam_toolbox builds the map from the robot's LiDAR while Nav2 drives the stations from
config/mission.yaml. The map is then saved with Nav2's map saver as <out>.pgm and <out>.yaml.
"""

import argparse
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def ensure_slam_active(node, settle_s=8.0):
    """Configure and activate slam_toolbox if its own launch left it unconfigured.

    Nav2's SLAM launch relies on an event to configure slam_toolbox, and in four of five launches
    that event never fired, so the map frame never appeared and Nav2's bringup timed out.
    """
    import time

    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    import rclpy

    get_state = node.create_client(GetState, '/slam_toolbox/get_state')
    change = node.create_client(ChangeState, '/slam_toolbox/change_state')
    if not get_state.wait_for_service(timeout_sec=60.0):
        raise RuntimeError('slam_toolbox is not running; was the sim started with slam:=True?')
    time.sleep(settle_s)  # let the launch's own configure event happen if it is going to

    def state():
        future = get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        return future.result().current_state.label

    for label, transition in (('unconfigured', Transition.TRANSITION_CONFIGURE),
                              ('inactive', Transition.TRANSITION_ACTIVATE)):
        if state() == label:
            future = change.call_async(ChangeState.Request(transition=Transition(id=transition)))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    print(f'slam_toolbox is {state()}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Build a depot map with SLAM.')
    parser.add_argument('--out', type=Path, default=REPO / 'maps' / 'depot_slam')
    parser.add_argument('--laps', type=int, default=1)
    args = parser.parse_args(argv)

    sys.path.insert(0, str(REPO))
    import rclpy
    from rclpy.parameter import Parameter

    from factoryrunner.mission_config import load_mission
    from factoryrunner.mission_controller import to_pose_stamped, warn_unless_cyclone
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    rclpy.init()
    navigator = BasicNavigator('factoryrunner_mapper')
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    warn_unless_cyclone(navigator)
    mission = load_mission(REPO / 'config' / 'mission.yaml')
    ensure_slam_active(navigator)
    # SLAM provides map -> odom from the start, so there is no AMCL and no initial pose to
    # wait for; 'robot_localization' is how BasicNavigator spells "skip the localizer".
    navigator.waitUntilNav2Active(localizer='robot_localization')

    failed = 0
    for lap in range(1, args.laps + 1):
        for station in mission.stations:
            navigator.goToPose(to_pose_stamped(navigator, station.pose, mission.frame_id))
            while not navigator.isTaskComplete():
                pass
            result = navigator.getResult()
            print(f'lap {lap} {station.name}: {result.name}')
            failed += result != TaskResult.SUCCEEDED

    args.out.parent.mkdir(parents=True, exist_ok=True)
    saved = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(args.out),
         '--ros-args', '-p', 'use_sim_time:=true'],
        capture_output=True, text=True, timeout=60)
    print(saved.stdout.strip().splitlines()[-1] if saved.stdout.strip() else saved.stderr)
    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if saved.returncode == 0 and failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
