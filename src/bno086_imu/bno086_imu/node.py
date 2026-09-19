"""BNO086 ROS 2 node. No synthetic IMU heartbeats after a stopped sensor stream."""
import math
import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32, UInt8
from .protocol import ACCEL, GYRO, MAG, ROTATION, ProtocolError, decode_reports, true_enu_quaternion
from .samples import Samples
from .transport import ShtpI2C


def covariance(stddev):
    v = stddev * stddev
    return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]


class Bno086Node(Node):
    def __init__(self):
        super().__init__('bno086_imu')
        defaults = dict(
            i2c_bus=1, i2c_address=0x4B, frame_id='imu_link',
            rate_hz=25.0, mag_rate_hz=10.0, declination_deg=0.0,
            sample_max_age=0.10, reconnect_timeout=2.0, min_accuracy=2,
            max_heading_error=0.35, orientation_stddev=0.05,
            angular_velocity_stddev=0.02, linear_acceleration_stddev=0.15,
            magnetic_stddev=0.000002,
        )
        self.p = {key: self.declare_parameter(
            key, value, ParameterDescriptor(read_only=True)).value
            for key, value in defaults.items()}
        self.validate()
        self.transport = None
        self.samples = Samples(self.p['sample_max_age'], self.p['min_accuracy'],
                               self.p['max_heading_error'])
        self.imu_pub = self.create_publisher(Imu, 'data', qos_profile_sensor_data)
        self.mag_pub = self.create_publisher(MagneticField, 'mag', qos_profile_sensor_data)
        self.accuracy_pub = self.create_publisher(UInt8, 'accuracy', qos_profile_sensor_data)
        self.azimuth_pub = self.create_publisher(Float32, 'azimuth', qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last_published = -float('inf')
        self.last_reason = 'waiting for calibrated Rotation Vector'
        try:
            self.transport = ShtpI2C(self.p['i2c_bus'], self.p['i2c_address'])
            self.transport.configure(self.p['rate_hz'], self.p['mag_rate_hz'])
        except Exception:
            self.close()
            raise
        self.started = self.last_poll = time.monotonic()
        self.last_sequences = {}
        self.create_timer(0.01, self.poll)
        self.create_timer(1.0, self.diagnostics)
        self.get_logger().info(
            f"BNO086 /dev/i2c-{self.p['i2c_bus']} address 0x{self.p['i2c_address']:02x}; "
            'waiting for fresh calibrated accel/gyro/Rotation Vector reports')

    def validate(self):
        if type(self.p['i2c_bus']) is not int or self.p['i2c_bus'] < 0:
            raise ValueError('i2c_bus must be a nonnegative integer')
        if type(self.p['i2c_address']) is not int or self.p['i2c_address'] not in (0x4A, 0x4B):
            raise ValueError('i2c_address must be 74 (0x4A) or 75 (0x4B)')
        if type(self.p['min_accuracy']) is not int or self.p['min_accuracy'] not in (1, 2, 3):
            raise ValueError('min_accuracy must be 1, 2 or 3; default 2')
        for key in ('rate_hz', 'mag_rate_hz'):
            if not math.isfinite(self.p[key]) or not 1 <= self.p[key] <= 100:
                raise ValueError(key + ' must be 1..100 Hz')
        for key in ('sample_max_age', 'reconnect_timeout', 'max_heading_error',
                    'orientation_stddev', 'angular_velocity_stddev',
                    'linear_acceleration_stddev', 'magnetic_stddev'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        if self.p['sample_max_age'] > 0.2:
            raise ValueError('sample_max_age must be <= 0.2 s for the navigation watchdog')
        if self.p['reconnect_timeout'] <= self.p['sample_max_age']:
            raise ValueError('reconnect_timeout must exceed sample_max_age')
        if not math.isfinite(self.p['declination_deg']) or not -180 <= self.p['declination_deg'] <= 180:
            raise ValueError('declination_deg must be finite, within -180..180')
        if not isinstance(self.p['frame_id'], str) or not self.p['frame_id'].strip():
            raise ValueError('frame_id must not be empty')

    def poll(self):
        begin = time.monotonic()
        # A stalled executor may have left old samples in the hardware queue.
        if begin - self.last_poll > 0.2:
            raise TimeoutError('IMU polling stalled; restart rather than re-stamp queued data')
        self.last_poll = begin
        for _ in range(32):
            item = self.transport.receive()
            if item is None:
                break
            channel, sequence, payload = item
            if channel == 1 and payload[:1] == b'\x01':
                raise ProtocolError('BNO086 reset at runtime: reinitialization required')
            if channel not in (3, 4):
                continue  # advertisements / feature acknowledgements / control
            if self.last_sequences.get(channel) == sequence:
                continue
            self.last_sequences[channel] = sequence
            for report in decode_reports(payload):
                received = time.monotonic()
                if not self.samples.add(report, received):
                    continue
                if report.sensor == ROTATION:
                    self.accuracy_pub.publish(UInt8(data=report.accuracy))
                if report.sensor == MAG and report.delay <= self.p['sample_max_age']:
                    self.publish_mag(report)
            if time.monotonic() - begin > self.p['sample_max_age']:
                raise TimeoutError('I2C drain too slow; refusing potentially queued IMU data')
        else:
            raise TimeoutError('SHTP input backlog; lower report rate/check bus')
        now = time.monotonic()
        missing = self.samples.missing(now, self.p['reconnect_timeout'], self.started)
        if missing:
            raise TimeoutError(f'Missing fresh SH-2 reports: {missing}')
        bundle = self.samples.take(now)
        if bundle is not None:
            self.publish_imu(bundle, now)

    def publish_imu(self, bundle, now):
        rotation = bundle[ROTATION][0]
        q = true_enu_quaternion(rotation.values, self.p['declination_deg'])
        msg = Imu()
        # Host receipt estimate, conservatively backdated to oldest component and
        # its report delay. SH-2 clock is NOT synchronized to ROS/UTC in this driver.
        oldest_age = max(now - received + report.delay for report, received in bundle.values())
        from rclpy.time import Time
        stamp_ns = self.get_clock().now().nanoseconds - int(oldest_age * 1e9)
        msg.header.stamp = Time(nanoseconds=max(0, stamp_ns)).to_msg()
        msg.header.frame_id = self.p['frame_id']
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = bundle[GYRO][0].values
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = bundle[ACCEL][0].values
        # SH-2 heading error is an estimate, not a guaranteed Gaussian sigma.
        # Use it conservatively as a covariance floor on all three axes.
        msg.orientation_covariance = covariance(max(
            self.p['orientation_stddev'], rotation.heading_accuracy))
        msg.angular_velocity_covariance = covariance(self.p['angular_velocity_stddev'])
        msg.linear_acceleration_covariance = covariance(self.p['linear_acceleration_stddev'])
        self.imu_pub.publish(msg)
        x, y, z, w = q
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        self.azimuth_pub.publish(Float32(data=(90.0 - math.degrees(yaw)) % 360.0))
        self.last_published = now

    def publish_mag(self, report):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['frame_id']
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = report.values
        msg.magnetic_field_covariance = covariance(self.p['magnetic_stddev'])
        self.mag_pub.publish(msg)

    def diagnostics(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = 'BNO086 IMU'
        status.hardware_id = f"i2c-{self.p['i2c_bus']}:0x{self.p['i2c_address']:02x}"
        healthy = now - self.last_published <= self.p['sample_max_age']
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        status.message = 'fresh calibrated IMU' if healthy else 'waiting for fresh/accurate IMU; no data published'
        if ROTATION in self.samples.latest:
            report, received = self.samples.latest[ROTATION]
            values = dict(rotation_accuracy=report.accuracy,
                          heading_error_rad=report.heading_accuracy,
                          rotation_age_s=round(now-received, 3))
            status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)
        if status.message != self.last_reason:
            self.get_logger().info(status.message)
            self.last_reason = status.message

    def close(self):
        if self.transport is not None:
            self.transport.close()
            self.transport = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = Bno086Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        code = 1
        if node is not None:
            node.get_logger().error(f'IMU stopped: {error}; launch will reconnect')
        else:
            print(f'BNO086 initialization failed: {error}', flush=True)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)
