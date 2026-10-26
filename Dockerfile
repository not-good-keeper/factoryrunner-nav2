# Headless FactoryRunner: Gazebo Harmonic, Nav2 and this package, no GUI. Results land in the
# mounted results/ directory.
#   docker build -t factoryrunner .
#   docker run --rm -v "$PWD/results:/ws/src/factoryrunner-nav2/results" factoryrunner
#   docker run --rm -v "$PWD/results:/ws/src/factoryrunner-nav2/results" factoryrunner trial_runner
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
        ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-nav2-minimal-tb4-sim ros-jazzy-ros-gz \
        ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge libopencv-dev python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Nav2's container froze under the default Fast DDS on the development machine.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ws
COPY . src/factoryrunner-nav2
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --base-paths src src/factoryrunner-nav2/factoryrunner_vision

WORKDIR /ws/src/factoryrunner-nav2
ENTRYPOINT ["bash", "scripts/docker_entrypoint.sh"]
CMD ["mission_controller"]
