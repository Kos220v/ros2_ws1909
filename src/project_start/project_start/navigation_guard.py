"""Fail-closed autonomous command gate. Not a certified emergency stop."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformListener, TransformException
from .safety import fresh, variance_ok


class NavigationGuard(Node):
    def __init__(self):
        super().__init__('navigation_guard')
        defaults = dict(gps_timeout=2.0, scan_timeout=0.4, imu_timeout=0.3,
                        odom_timeout=0.4, max_gps_stddev=1.5,
                        max_global_stddev=2.0, command_timeout=0.3)
        self.p = {}
        for key, value in defaults.items():
            self.p[key] = float(self.declare_parameter(key, value).value)
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.samples = {}
        self.command = (Twist(), -float('inf'))
        self.mission = (False, -float('inf'))
        self.mode = 1
        self.mode_time = -float('inf')
        self.was_active = False
        self.fault = False
        self.previous_reason = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for key, cls, topic in [('gps', NavSatFix, '/gps/fix'), ('scan', LaserScan, '/scan'),
                                ('imu', Imu, '/imu/data'), ('wheel', Odometry, '/odom/vesc'),
                                ('local', Odometry, '/odometry/local'),
                                ('global', Odometry, '/odometry/global')]:
            self.create_subscription(cls, topic, lambda msg, k=key: self.record(k, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel/checked', self.on_command, 10)
        self.create_subscription(Bool, '/navigation/mission_enabled', self.on_mission, 10)
        self.create_subscription(Int8, '/control_mode', self.on_mode, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel/auto', 10)
        self.ready_pub = self.create_publisher(Bool, '/navigation/ready', 10)
        self.create_timer(0.05, self.tick)

    def record(self, key, msg):
        self.samples[key] = (msg, time.monotonic())

    def on_mission(self, msg):
        self.mission = (msg.data, time.monotonic())

    def on_command(self, msg):
        self.command = (msg, time.monotonic())

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.command = (Twist(), -float('inf'))
        self.mode, self.mode_time = msg.data, time.monotonic()
        if self.mode == 1:
            self.fault = self.was_active = False

    def health(self, now):
        ros_now = self.get_clock().now().nanoseconds / 1e9
        for key in ('gps', 'scan', 'imu', 'wheel', 'local', 'global'):
            if key not in self.samples:
                return 'waiting for ' + key
            msg, received = self.samples[key]
            timeout = self.p[key + '_timeout'] if key in ('gps', 'scan', 'imu') else self.p['odom_timeout']
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if stamp <= 0 or not fresh(now - received, ros_now - stamp, timeout):
                return 'stale ' + key
        gps = self.samples['gps'][0]
        if (gps.status.status < 0 or gps.position_covariance_type == 0
                or not all(math.isfinite(v) for v in (gps.latitude, gps.longitude, gps.altitude))
                or not variance_ok([gps.position_covariance[0], gps.position_covariance[4]],
                                   self.p['max_gps_stddev'])):
            return 'invalid GPS fix/covariance'
        imu = self.samples['imu'][0]
        q = imu.orientation
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if (not math.isfinite(norm) or not 0.9 <= norm <= 1.1
                or imu.orientation_covariance[0] < 0 or not math.isfinite(imu.angular_velocity.z)):
            return 'invalid IMU'
        if not variance_ok([self.samples['wheel'][0].twist.covariance[0]], 10.0):
            return 'invalid wheel telemetry'
        cov = self.samples['global'][0].pose.covariance
        if not variance_ok([cov[0], cov[7]], self.p['max_global_stddev']):
            return 'global localization uncertainty'
        scan = self.samples['scan'][0]
        if not scan.ranges or not any(
                r == float('inf') or (math.isfinite(r) and scan.range_min <= r <= scan.range_max)
                for r in scan.ranges):
            return 'empty/invalid scan'
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            try:
                tf = self.tf.lookup_transform(parent, child, Time())
                age = ros_now - Time.from_msg(tf.header.stamp).nanoseconds / 1e9
                if not -0.1 <= age <= self.p['odom_timeout']:
                    return 'stale TF ' + parent
            except TransformException:
                return 'missing TF ' + parent
        return ''

    def tick(self):
        now = time.monotonic()
        reason = self.health(now)
        mode_ok = self.mode == 0 and 0 <= now - self.mode_time < 0.5
        if self.was_active and (reason or not mode_ok) and self.mode != 1:
            self.fault = True
        if mode_ok and not reason and not self.fault:
            self.was_active = True
        ready = not reason and not self.fault
        self.ready_pub.publish(Bool(data=ready))
        command, received = self.command
        mission_ok = self.mission[0] and 0 <= now - self.mission[1] < 0.5
        valid_cmd = all(math.isfinite(v) for v in (
            command.linear.x, command.linear.y, command.linear.z,
            command.angular.x, command.angular.y, command.angular.z))
        self.pub.publish(command if ready and mode_ok and mission_ok and valid_cmd
                         and 0 <= now - received < self.p['command_timeout'] else Twist())
        reason = reason or ('latched fault: select MANUAL to reset' if self.fault else 'ready')
        if reason != self.previous_reason:
            self.get_logger().info(reason)
            self.previous_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
