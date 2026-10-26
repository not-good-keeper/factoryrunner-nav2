#!/usr/bin/env bash
# One-time setup for a fresh Ubuntu 24.04 (WSL2) machine. Run with: bash scripts/setup_jazzy.sh
set -euo pipefail

if ! grep -q 'VERSION_CODENAME=noble' /etc/os-release; then
    echo "This needs Ubuntu 24.04 (noble); ROS 2 Jazzy has no binaries for $(. /etc/os-release; echo "$PRETTY_NAME")." >&2
    exit 1
fi

sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
fi

sudo apt-get install -y \
    ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-nav2-minimal-tb4-sim ros-jazzy-ros-gz \
    ros-jazzy-nav2-simple-commander ros-dev-tools \
    python3-pytest python3-flake8 python3-matplotlib python3-pandas mesa-utils git

grep -qx 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo
echo "OpenGL renderer seen by WSLg:"
glxinfo -B 2>/dev/null | grep -E 'Device|Accelerated|OpenGL renderer' || echo "  glxinfo failed; is WSLg running?"
echo "If it says llvmpipe, try: echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc"
