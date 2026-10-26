"""Watch the running stack and publish what is wrong on /diagnostics.

ros2 run factoryrunner health_monitor

Checks, once a second: how long since map->odom last arrived on /tf, the /scan rate, the
lifecycle state of the main Nav2 nodes, and the collision monitor's action. Every change of
level is logged, so a run's log shows when something went wrong. The checks themselves live in
factoryrunner/health.py.
"""

import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from factoryrunner import health
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

WATCHED_NODES = ['amcl', 'map_server', 'planner_server', 'controller_server',
                 'velocity_smoother', 'collision_monitor', 'bt_navigator']


class HealthMonitor(Node):

    def __init__(self):
        super().__init__('health_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.expected_scan_hz = self.declare_parameter('expected_scan_hz', 5.0).value
        self.tf_warn_s = self.declare_parameter('tf_warn_s', 1.0).value
        self.tf_error_s = self.declare_parameter('tf_error_s', 2.5).value
        # Long enough for the first scans, transforms and lifecycle answers to arrive.
        self.startup_grace_s = self.declare_parameter('startup_grace_s', 3.0).value
        nodes = self.declare_parameter('nodes', WATCHED_NODES).value

        self._scan_stamps = []
        self._last_map_odom = None
        self._monitor_action = None
        self._levels = {}
        self._states = {n: None for n in nodes}
        self._state_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in nodes}
        self._pending = {}

        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self._on_monitor, 10)
        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._tick)
        self._started = None
        self._poll_lifecycle()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, _msg):
        now = self._now()
        self._scan_stamps.append(now)
        self._scan_stamps = [t for t in self._scan_stamps if now - t < 5.0]

    def _on_tf(self, msg):
        if any(t.header.frame_id == 'map' and t.child_frame_id == 'odom' for t in msg.transforms):
            self._last_map_odom = self._now()

    def _on_monitor(self, msg):
        self._monitor_action = msg.action_type

    def _poll_lifecycle(self):
        for node, client in self._state_clients.items():
            pending = self._pending.pop(node, None)
            if pending is not None and not pending.done():
                # Still unanswered after a full period: the node, or its process, is stuck.
                client.remove_pending_request(pending)
                self._states[node] = None
            if not client.service_is_ready():
                self._states[node] = None
                continue
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda f, n=node: self._on_state(n, f))
            self._pending[node] = future

    def _on_state(self, node, future):
        if future.cancelled() or future.exception() is not None:
            return
        self._states[node] = future.result().current_state.label

    def _tick(self):
        now = self._now()
        if self._started is None:
            self._started = now
        warming_up = now - self._started < self.startup_grace_s
        tf_age = None if self._last_map_odom is None else now - self._last_map_odom
        checks = {
            'localization (map->odom)': health.tf_level(tf_age, self.tf_warn_s, self.tf_error_s),
            'lidar (/scan)': health.scan_level(
                health.rate_hz(self._scan_stamps, now, 2.0), self.expected_scan_hz),
            'nav2 lifecycle': health.lifecycle_level(dict(self._states)),
            'collision monitor': health.monitor_level(self._monitor_action),
        }
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, (level, text) in checks.items():
            if warming_up and level != health.OK:
                level, text = health.STALE, f'starting up ({text})'
            status = DiagnosticStatus(level=bytes([level]), name=f'factoryrunner: {name}',
                                      message=text, hardware_id='turtlebot3_waffle')
            if name == 'nav2 lifecycle':
                status.values = [KeyValue(key=n, value=s or 'no answer')
                                 for n, s in sorted(self._states.items())]
            msg.status.append(status)
            if self._levels.get(name) != level:
                self._report(name, level, text)
                self._levels[name] = level
        self._pub.publish(msg)
        self._poll_lifecycle()

    def _report(self, name, level, text):
        line = f'{name}: {health.LEVEL_NAMES[level]} - {text}'
        if level == health.ERROR:
            self.get_logger().error(line)
        elif level == health.WARN:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)


def main(argv=None):
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
