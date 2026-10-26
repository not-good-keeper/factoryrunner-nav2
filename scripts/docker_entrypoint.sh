#!/usr/bin/env bash
# Start the headless sim, wait for localization, run one factoryrunner program, then stop.
#   docker_entrypoint.sh [mission_controller|trial_runner] [--ros-args ...]
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

log=$(mktemp)
ros2 launch factoryrunner sim.launch.py headless:=True use_rviz:=False > "$log" 2>&1 &
sim=$!
stop_sim() {
    # A background job in a non-interactive shell ignores SIGINT, so ask with SIGTERM, and
    # do not wait for ever: this script is the container's PID 1, so whatever is still
    # running when it exits is torn down with the container.
    kill -TERM "$sim" 2>/dev/null || return 0
    for _ in $(seq 1 15); do
        kill -0 "$sim" 2>/dev/null || return 0
        sleep 1
    done
}
trap stop_sim EXIT

# The first launch also downloads the depot model from Gazebo Fuel, so allow a few minutes.
for _ in $(seq 1 180); do
    if grep -q 'lifecycle_manager_localization.*Managed nodes are active' "$log"; then
        break
    fi
    if ! kill -0 "$sim" 2>/dev/null; then
        echo 'simulation exited during startup:' >&2
        tail -40 "$log" >&2
        exit 1
    fi
    sleep 1
done
if ! grep -q 'lifecycle_manager_localization.*Managed nodes are active' "$log"; then
    echo 'localization did not come up within 180 s:' >&2
    tail -40 "$log" >&2
    exit 1
fi

program=${1:-mission_controller}
shift || true
status=0
ros2 run factoryrunner "$program" "$@" || status=$?
exit "$status"
