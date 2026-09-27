"""Explicit mode gating: loss of manual commands never engages AUTO."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from .policy import select_source


class CmdMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_mux_node')
        self.mode = 1
        self.mode_time = -float('inf')
        self.commands = {}
        self.create_subscription(Int8, '/control_mode', self.on_mode, 10)
        for source in ('manual', 'app_manual', 'auto'):
            self.create_subscription(
                Twist, '/cmd_vel/' + source,
                lambda msg, source=source: self.on_command(source, msg), 10)
        self.pub_final = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.02, self.publish_logic)

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.commands.clear()
            self.pub_final.publish(Twist())
        self.mode, self.mode_time = msg.data, time.monotonic()

    def on_command(self, source, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.commands[source] = (
            msg if all(math.isfinite(v) for v in values) else Twist(), time.monotonic())

    def publish_logic(self):
        now = time.monotonic()
        source = select_source(self.mode, now - self.mode_time,
                               {k: now - v[1] for k, v in self.commands.items()})
        self.pub_final.publish(self.commands[source][0] if source else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_final.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
