"""Execute actual adapter methods with fake ROS messages/clock (no DDS or hardware)."""
import ast
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace as NS
from project_start.odometry_math import CounterOdometry, OdometryError


class FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return NS(nanoseconds=self.nanoseconds)

    @classmethod
    def from_msg(cls, msg):
        return cls(msg.nanoseconds)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))

    sendTransform = publish


def odometry():
    return NS(header=NS(), pose=NS(pose=NS(position=NS(), orientation=NS())),
              twist=NS(twist=NS(linear=NS(), angular=NS())))


def transform():
    return NS(transform=NS(translation=NS(), rotation=NS()))


def adapter():
    path = Path(__file__).parents[1] / 'project_start/counter_odometry.py'
    source = ast.parse(path.read_text())
    # Only strip ROS imports/entry point, retain actual class/method definitions.
    definitions = [n for n in source.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
                   and n.name != 'main']
    now = [1.12]
    scope = dict(Node=object, math=math, Time=FakeTime, Odometry=odometry,
                 TransformStamped=transform, Bool=lambda **kw: NS(**kw),
                 OdometryError=OdometryError, time=NS(monotonic=lambda: now[0]))
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), scope)
    node = object.__new__(scope['CounterOdometryNode'])
    node.p = dict(input_timeout=.25, odom_frame='odom', base_frame='base_link',
                  left_time_offset=0., right_time_offset=0.)
    node.model = CounterOdometry()
    for t, ticks in ((1., 0), (1.1, 100)):
        for side in ('left','right'):
            node.model.push_track(side,t,ticks,.001,'s')
        node.model.push_imu(t,0,.0025,0,.0004)
    node.received = {key: (1.1,1.1) for key in ('left','right','imu')}
    node.pub, node.tf_pub, node.health_pub = Publisher(), Publisher(), Publisher()
    node.fault, node.last_result, node.last_published = '', None, -float('inf')
    node.get_clock = lambda: NS(now=lambda: FakeTime(round(now[0]*1e9)))
    node.get_logger = lambda: NS(error=lambda _: None)
    return node, now


def test_measurement_stamp_and_covariance_contract():
    node, _ = adapter()
    node.tick()
    msg = node.pub.messages[-1]
    assert msg.header.stamp.nanoseconds == 1_100_000_000  # not receipt/publication time 1.12
    assert msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
    assert abs(msg.pose.pose.position.x-.1) < 1e-10
    assert msg.pose.covariance[14] == 1e6  # z not measured
    assert msg.twist.covariance[7] == 1e6  # vy not measured
    assert len(msg.pose.covariance) == len(msg.twist.covariance) == 36
    assert node.tf_pub.messages[-1].header.stamp.nanoseconds == 1_100_000_000
    assert node.health_pub.messages[-1].data is True


def test_no_republication_without_new_measurements():
    node, now = adapter()
    node.tick()
    now[0] = 1.13
    node.tick()
    assert len(node.pub.messages) == len(node.tf_pub.messages) == 1


def test_dropout_latches_and_stops_tf_even_after_heartbeat_returns():
    node, now = adapter()
    node.tick()
    now[0] = 1.5
    node.tick()
    assert node.fault and not node.health_pub.messages[-1].data
    node.received = {key:(1.5,1.5) for key in ('left','right','imu')}
    node.tick()
    assert len(node.pub.messages) == len(node.tf_pub.messages) == 1
    assert not node.health_pub.messages[-1].data


def test_invalid_track_after_start_is_not_zero_movement():
    node, _ = adapter()
    node.tick()
    node.on_track('left', NS(valid=False))
    assert node.fault
    assert not node.health_pub.messages[-1].data
    node.tick()
    assert len(node.pub.messages) == 1
