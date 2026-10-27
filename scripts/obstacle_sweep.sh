#!/usr/bin/env bash
# Drop the pallet at shrinking distances and log every lap. Needs the sim running,
# the robot at the dock and Nav2 up, as for trial_runner:
#   bash scripts/obstacle_sweep.sh [laps per distance] [distances...]
#   bash scripts/obstacle_sweep.sh 3 3.0 2.0 1.5 1.2
# Each distance gets its own trials CSV under results/obstacle_sweep_<timestamp>/d<distance>/.
set -euo pipefail

laps=${1:-3}
shift || true
distances=("${@:-3.0 2.0 1.5 1.2}")
read -r -a distances <<< "${distances[*]}"

repo=$(cd "$(dirname "$0")/.." && pwd)
out="$repo/results/obstacle_sweep_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out"
obstacle_laps=$(seq -s ', ' 1 "$laps")

for d in "${distances[@]}"; do
    mission="$out/mission_d$d.yaml"
    # Same stations and pallet spot as config/mission.yaml; only the laps and the drop
    # distance change.
    sed -e "s/^  laps: .*/  laps: $laps/" \
        -e "s/^  obstacle_laps: .*/  obstacle_laps: [$obstacle_laps]/" \
        -e "s/spawn_distance: [0-9.]*/spawn_distance: $d/" \
        "$repo/config/mission.yaml" > "$mission"
    echo "== drop distance $d m, $laps laps"
    ros2 run factoryrunner trial_runner --ros-args \
        -p mission_file:="$mission" -p results_dir:="$out/d$d"
done
echo "sweep results in $out"
