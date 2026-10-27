"""Move the spawned pallet across the aisle while the robot drives.

The pallet is a static model, so it is teleported in small steps through the world's
set_pose service rather than simulated with physics. Requests go through Gazebo's in-process
Python transport, about half a millisecond each; the gz CLI takes over half a second per call.
"""

import math
import threading

from factoryrunner.obstacle import map_to_gazebo, SPAWN_Z, WORLD

UPDATE_HZ = 20.0


def crossing_positions(start, end, speed, rate_hz=UPDATE_HZ):
    """Map-frame (x, y) points from start to end at `speed` m/s, one per 1/rate_hz seconds."""
    if speed <= 0 or rate_hz <= 0:
        raise ValueError('speed and rate_hz must be positive')
    steps = max(1, math.ceil(math.dist(start, end) / speed * rate_hz))
    return [(start[0] + (end[0] - start[0]) * i / steps,
             start[1] + (end[1] - start[1]) * i / steps) for i in range(steps + 1)]


class CrossingMover(threading.Thread):
    """Walks a model through `positions` at UPDATE_HZ on its own thread, until done or stopped."""

    def __init__(self, name, positions, world=WORLD):
        super().__init__(daemon=True)
        self._name = name
        self._positions = positions
        self._service = f'/world/{world}/set_pose'
        # Not self._stop: threading.Thread uses that name internally and join() calls it.
        self._halt = threading.Event()
        self.failed_requests = 0

    def stop(self):
        self._halt.set()

    def run(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node

        node = Node()
        for x, y in self._positions:
            if self._halt.is_set():
                return
            req = Pose()
            req.name = self._name
            req.position.x, req.position.y = map_to_gazebo(x, y)
            req.position.z = SPAWN_Z['pallet_box']
            req.orientation.w = 1.0
            ok, _ = node.request(self._service, req, Pose, Boolean, 1000)
            if not ok:
                self.failed_requests += 1
            # Wall-clock pacing; the sim runs close to real time on this setup.
            self._halt.wait(1.0 / UPDATE_HZ)


def wall_seconds_for(positions):
    return (len(positions) - 1) / UPDATE_HZ
