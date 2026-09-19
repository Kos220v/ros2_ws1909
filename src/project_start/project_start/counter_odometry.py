"""Synchronized VESC-counter/BNO086 local odometry; sole odom->base_link TF owner."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tracked_robot_interfaces.msg import TrackTicks
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from .odometry_math import CounterOdometry, OdometryError, body_orientation_and_rate


def ros_covariance(cov3):
    cov = [0.0]*36
    for i in range(6):
        cov[6*i+i] = 1e6
    for i, row in enumerate((0, 1, 5)):
        for j, col in enumerate((0, 1, 5)):
            cov[6*row+col] = float(cov3[i, j])
    return cov


class CounterOdometryNode(Node):
    def __init__(self):
        super().__init__('counter_odometry')
        model = dict(max_gap=0.15, max_track_speed=2.0, max_yaw_rate=2.0,
                     yaw_jump_tolerance=0.08, track_width=0.48,
                     distance_stddev_per_meter=0.03, distance_noise_floor=0.001,
                     lateral_stddev_per_meter=0.03, lateral_stddev_per_radian=0.02,
                     slip_noise_gain=0.3, heading_bias_stddev=0.05, timestamp_stddev=0.005,
                     min_update_interval=0.02)
        runtime = dict(odom_frame='odom', base_frame='base_link', input_timeout=0.25,
                       max_tilt=0.35, left_time_offset=0.0, right_time_offset=0.0,
                       imu_time_offset=0.0)
        self.p = {name: self.declare_parameter(name, value,
                  ParameterDescriptor(read_only=True)).value for name, value in (model | runtime).items()}
        self.model = CounterOdometry(**{k: self.p[k] for k in model})
        if not 0 < self.p['input_timeout'] <= 0.4 or not 0 < self.p['max_tilt'] < 1.0:
            raise ValueError('input_timeout must be <=0.4s, max_tilt must be <1 rad')
        if self.p['input_timeout'] < self.p['max_gap']:
            raise ValueError('input_timeout must not be smaller than max_gap')
        for name in ('left_time_offset', 'right_time_offset', 'imu_time_offset'):
            if not math.isfinite(self.p[name]) or abs(self.p[name]) > 0.1:
                raise ValueError(name + ' must be finite and within +/-0.1 seconds')
        self.received = {}
        self.fault = ''
        self.last_result = None
        self.last_published = -float('inf')
        self.mount = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odometry/local', 20)
        self.health_pub = self.create_publisher(Bool, '/odometry/healthy', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        for side in ('left', 'right'):
            self.create_subscription(TrackTicks, '/kolesa/track_'+side,
                                     lambda msg, side=side: self.on_track(side, msg), 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)
        self.create_timer(1.0, self.diagnostics)

    def fail(self, reason):
        if not self.fault:
            self.fault = reason
            self.get_logger().error(reason + '; stop, investigate, restart localization at standstill')
        self.health_pub.publish(Bool(data=False))

    def check_stamp(self, msg, stream):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds/1e9 + self.p[stream+'_time_offset']
        now = self.get_clock().now().nanoseconds/1e9
        if stamp <= 0 or not -0.02 <= now-stamp <= self.p['input_timeout']:
            raise OdometryError('stale/future timestamp: '+stream)
        return stamp

    def on_track(self, side, msg):
        if self.fault:
            return
        try:
            if not msg.valid:
                if self.model.time is not None:
                    self.fail(side + ' VESC sample invalid')
                return
            if msg.header.frame_id != self.p['base_frame']:
                raise OdometryError('track base frame mismatch')
            stamp = self.check_stamp(msg, side)
            if self.model.push_track(side, stamp, msg.position_ticks, msg.meters_per_tick, msg.session_id):
                self.received[side] = (stamp, time.monotonic())
        except OdometryError as error:
            self.fail(str(error))

    def on_imu(self, msg):
        if self.fault:
            return
        try:
            stamp = self.check_stamp(msg, 'imu')
            if msg.orientation_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0:
                raise OdometryError('IMU lacks orientation or gyro')
            if not msg.header.frame_id:
                raise OdometryError('IMU frame empty')
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], msg.header.frame_id, Time())
            q_mount = tf.transform.rotation
            mount = (q_mount.x, q_mount.y, q_mount.z, q_mount.w)
            if self.mount is not None and min(
                    sum((a-b)**2 for a, b in zip(mount, self.mount)),
                    sum((a+b)**2 for a, b in zip(mount, self.mount))) > 1e-8:
                raise OdometryError('IMU mounting TF changed during odometry')
            self.mount = mount
            q, w = msg.orientation, msg.angular_velocity
            yaw, rate = body_orientation_and_rate((q.x, q.y, q.z, q.w), mount,
                                                  (w.x, w.y, w.z), self.p['max_tilt'])
            orientation_cov = [msg.orientation_covariance[i] for i in (0, 4, 8)]
            rate_cov = [msg.angular_velocity_covariance[i] for i in (0, 4, 8)]
            if any(not math.isfinite(v) or v <= 0 for v in orientation_cov+rate_cov):
                raise OdometryError('IMU covariance must be positive and finite')
            # Conservative for the isotropic BNO086 covariances; account for tilt
            # amplification of the Euler yaw rate and host timing uncertainty.
            yaw_var = max(orientation_cov)/math.cos(self.p['max_tilt'])**2
            yaw_var += (rate*self.p['timestamp_stddev'])**2
            rate_var = max(rate_cov)/math.cos(self.p['max_tilt'])**2
            if self.model.push_imu(stamp, yaw, yaw_var, rate, rate_var):
                self.received['imu'] = (stamp, time.monotonic())
        except TransformException:
            if self.model.time is not None:
                self.fail('IMU mounting TF unavailable')
        except OdometryError as error:
            self.fail(str(error))

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds/1e9
        if not self.fault:
            if self.model.time is not None:
                for stream in ('left', 'right', 'imu'):
                    stamp, receipt = self.received.get(stream, (0, -float('inf')))
                    if (not 0 <= now-receipt <= self.p['input_timeout']
                            or not -0.02 <= ros_now-stamp <= self.p['input_timeout']):
                        self.fail('missing/stale '+stream)
                        break
            if not self.fault:
                try:
                    result = self.model.advance()
                    if result is not None:
                        self.publish(result)
                        self.last_result = result
                        self.last_published = now
                except OdometryError as error:
                    self.fail(str(error))
        healthy = not self.fault and now-self.last_published <= self.p['input_timeout']
        self.health_pub.publish(Bool(data=healthy))

    def publish(self, result):
        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=round(result.stamp*1e9)).to_msg()
        msg.header.frame_id = self.p['odom_frame']
        msg.child_frame_id = self.p['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(result.x), float(result.y)
        msg.pose.pose.orientation.z = math.sin(result.yaw/2)
        msg.pose.pose.orientation.w = math.cos(result.yaw/2)
        msg.pose.covariance = ros_covariance(result.covariance)
        msg.twist.twist.linear.x = float(result.vx)
        msg.twist.twist.angular.z = float(result.wz)
        cov = [0.0]*36
        for i in range(6):
            cov[6*i+i] = 1e6
        cov[0], cov[35] = float(result.vx_variance), float(result.wz_variance)
        msg.twist.covariance = cov  # lateral speed is NOT a trusted zero measurement
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = float(result.x)
        tf.transform.translation.y = float(result.y)
        tf.transform.rotation = msg.pose.pose.orientation
        self.pub.publish(msg)
        self.tf_pub.sendTransform(tf)  # measurement time only, no extrapolation/re-stamping

    def diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'Counter + BNO086 odometry'
        status.hardware_id = 'tracked_robot'
        status.level = DiagnosticStatus.ERROR if self.fault else (
            DiagnosticStatus.OK if self.last_result else DiagnosticStatus.WARN)
        status.message = self.fault or ('synchronized counters + heading' if self.last_result
                                       else 'waiting for common time interval / IMU mounting TF')
        if self.last_result:
            status.values = [KeyValue(key='track_vs_imu_residual_m', value=str(self.last_result.slip_residual)),
                             KeyValue(key='stamp', value=str(self.last_result.stamp))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = CounterOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.health_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
