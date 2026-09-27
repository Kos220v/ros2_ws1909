"""Sequential WGS84 route via robot_localization FromLL and Nav2 NavigateToPose."""
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformListener
from .safety import validate_route, check_legs


class GpsRoute(Node):
    def __init__(self):
        super().__init__('gps_route')
        self.file = self.declare_parameter('route_file', '').value
        self.start_timeout = self.positive('start_timeout', 120.0)
        self.goal_timeout = self.positive('goal_timeout', 180.0)
        self.max_leg = self.positive('max_leg_m', 15.0)
        if self.max_leg > 15.0:
            raise ValueError('max_leg_m must be <= 15 for the supplied 60 m costmap')
        self.ready = (False, -float('inf'))
        self.mode = (1, -float('inf'))
        self.motion_enabled = False
        self.running = False
        self.interrupted = False
        self.handle = None
        self.create_subscription(Bool, '/navigation/ready', self.on_ready, 10)
        self.create_subscription(Int8, '/control_mode', self.on_mode, 10)
        self.permission = self.create_publisher(Bool, '/navigation/mission_enabled', 10)
        self.create_timer(0.1, self.heartbeat)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def positive(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be finite and positive')
        return value

    def on_ready(self, msg):
        self.ready = (msg.data, time.monotonic())
        if self.running and not msg.data:
            self.interrupted = True

    def on_mode(self, msg):
        self.mode = (msg.data, time.monotonic())
        if self.running and msg.data != 0:
            self.interrupted = True

    def safe(self):
        now = time.monotonic()
        return (not self.interrupted and self.ready[0] and self.mode[0] == 0
                and 0 <= now - self.ready[1] < 0.5 and 0 <= now - self.mode[1] < 0.5)

    def heartbeat(self):
        if self.running and not self.safe():
            self.interrupted = True
        self.permission.publish(Bool(data=self.running and self.motion_enabled and self.safe()))

    def wait(self, future, timeout, check_safety=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if check_safety and not self.safe():
                raise RuntimeError('Mission interrupted: mode/sensor heartbeat lost')
            if time.monotonic() > deadline:
                raise TimeoutError('ROS service/action response timeout')
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')
        return future.result()

    def run(self):
        with open(self.file, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        points = validate_route(data)
        if data.get('enabled') is not True:
            raise ValueError('Set enabled: true only after checking your own route')
        self.get_logger().info('Waiting for localization, Nav2, and explicit AUTO mode')
        deadline = time.monotonic() + self.start_timeout
        while not (self.safe() and self.from_ll.service_is_ready()
                   and self.navigator.server_is_ready()):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise TimeoutError('Readiness timeout; check guard log and AUTO mode')
        # Mission permission stays OFF while all route coordinates are validated.
        mapped = []
        for lat, lon, alt in points:
            request = FromLL.Request()
            request.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=alt)
            result = self.wait(self.from_ll.call_async(request), 5.0)
            mapped.append((result.map_point.x, result.map_point.y))
        transform = self.tf.lookup_transform('map', 'base_link', Time())
        start = (transform.transform.translation.x, transform.transform.translation.y)
        check_legs(start, mapped, self.max_leg)
        self.running = True
        previous = start
        for i, (x, y) in enumerate(mapped):
            if not self.safe():
                raise RuntimeError('Mission no longer safe')
            # Use incoming segment heading: avoids turning before reaching a corner.
            yaw = math.atan2(y - previous[1], x - previous[0])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal = NavigateToPose.Goal()
            goal.pose = pose
            self.get_logger().info(f'Waypoint {i+1}/{len(mapped)}: map ({x:.2f}, {y:.2f})')
            self.motion_enabled = False
            self.permission.publish(Bool(data=False))
            # Collect goal handle even if safety changes during acceptance, then cancel.
            self.handle = self.wait(self.navigator.send_goal_async(goal), 10.0, False)
            if not self.handle.accepted:
                raise RuntimeError('Nav2 rejected goal')
            if not self.safe():
                raise RuntimeError('Safety changed while accepting goal')
            self.motion_enabled = True
            result = self.wait(self.handle.get_result_async(), self.goal_timeout)
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Nav2 failed waypoint {i+1}: status {result.status}; route stopped')
            self.handle = None
            previous = (x, y)
        self.get_logger().info('Route complete')

    def stop(self):
        self.motion_enabled = False
        self.running = False
        self.permission.publish(Bool(data=False))
        if self.handle is not None and self.handle.accepted:
            try:
                self.wait(self.handle.cancel_goal_async(), 3.0, False)
            except (RuntimeError, TimeoutError) as error:
                self.get_logger().error(f'Cancellation failed: {error}; mission gate is OFF')


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = GpsRoute()
        node.run()
    except KeyboardInterrupt:
        code = 130
    except Exception as error:
        code = 1
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(error)
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)
