#!/usr/bin/env bash
# Local stand-in for .github/workflows/ci.yml while Actions is disabled on this account.
# Copies the files git would commit (tracked plus new, unignored ones) into a throwaway
# workspace and runs the same lint, build and test steps there, so a stray untracked or
# ignored file in this checkout cannot make it pass. Run before every commit:
#   bash scripts/check.sh
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
ws=$(mktemp -d "${TMPDIR:-/tmp}/fr_check.XXXXXX")
trap 'rm -rf "$ws"' EXIT
pkg="$ws/src/factoryrunner-nav2"
mkdir -p "$pkg"

cd "$repo"
git ls-files -z --cached --others --exclude-standard | while IFS= read -r -d '' f; do
    # staged deletions are still listed by --cached
    [ -e "$f" ] && cp --parents -- "$f" "$pkg/"
done

set +u  # the ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
set -u
cd "$ws"

echo '== lint'
# Same as ci.yml: flake8 7 only reads config from the directory it runs in.
python3 -m flake8 --config "$pkg/setup.cfg" src/factoryrunner-nav2

echo '== build'
colcon build --symlink-install --event-handlers console_cohesion- summary+

echo '== unit tests'
set +u
source install/setup.bash
set -u
python3 -m pytest -q src/factoryrunner-nav2/test

echo '== all checks passed'
