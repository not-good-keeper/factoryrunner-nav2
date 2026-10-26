# FactoryRunner

TurtleBot3 (Waffle) running Nav2 in a simulated factory depot. Work in progress; this README gets its full write-up once the trial results exist.

## Requirements

- Ubuntu 24.04 (native or WSL2), ROS 2 Jazzy, Gazebo Harmonic
- `bash scripts/setup_jazzy.sh` installs everything needed on a fresh 24.04 machine
- Cyclone DDS as the ROS 2 middleware: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every terminal (the setup script adds it to `~/.bashrc`). Under the default Fast DDS the Nav2 container froze mid-mission; see [docs/CHALLENGES.md](docs/CHALLENGES.md).

## Build

```bash
mkdir -p ~/fr_ws/src && cd ~/fr_ws/src
git clone <repo-url> factoryrunner-nav2
cd ~/fr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # in each terminal, unless ~/.bashrc has it

# Gazebo + RViz + Nav2 in the depot world
ros2 launch factoryrunner sim.launch.py

# in a second terminal (same export), once Nav2 is up: drive the station loop
ros2 run factoryrunner mission_controller
```

## Tests

```bash
python3 -m pytest -q test/
```
