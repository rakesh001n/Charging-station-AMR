#!/usr/bin/env python3

import math

import rclpy
from battery_interfaces.msg import BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.declare_parameter('initial_level', 100.0)
        self.declare_parameter('distance_drain_per_meter', 2.0)
        self.declare_parameter('idle_drain_per_second', 0.03)
        self.declare_parameter('charge_rate_per_second', 12.0)
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('docking_distance', 0.30)
        self.declare_parameter('charging_enabled', True)

        self.level = float(self.get_parameter('initial_level').value)
        self.distance_drain = float(self.get_parameter('distance_drain_per_meter').value)
        self.idle_drain = float(self.get_parameter('idle_drain_per_second').value)
        self.charge_rate = float(self.get_parameter('charge_rate_per_second').value)
        self.charger_x = float(self.get_parameter('charger_x').value)
        self.charger_y = float(self.get_parameter('charger_y').value)
        self.docking_distance = float(self.get_parameter('docking_distance').value)
        self.last_position = None
        self.last_update = self.get_clock().now()
        self.charging_enabled = bool(self.get_parameter('charging_enabled').value)
        self.docked = False

        self.publisher = self.create_publisher(BatteryState, '/battery_state', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/battery_charging_enable', self.charge_callback, 10)
        self.create_subscription(Twist, '/raw_cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info(f'Battery node started at {self.level:.1f}%')

    def charge_callback(self, message):
        self.charging_enabled = message.data

    def cmd_callback(self, message):
        if self.level > 0.0:
            self.cmd_publisher.publish(message)
        else:
            self.cmd_publisher.publish(Twist())

    def odom_callback(self, message):
        position = message.pose.pose.position
        if self.last_position is not None:
            distance = math.hypot(position.x - self.last_position[0], position.y - self.last_position[1])
            self.level -= distance * self.distance_drain
        self.last_position = (position.x, position.y)
        self.docked = math.hypot(position.x - self.charger_x, position.y - self.charger_y) <= self.docking_distance

    def update(self):
        now = self.get_clock().now()
        elapsed = max(0.0, (now - self.last_update).nanoseconds / 1e9)
        self.last_update = now
        charging = self.charging_enabled and self.docked
        self.level += (self.charge_rate if charging else -self.idle_drain) * elapsed
        self.level = max(0.0, min(100.0, self.level))

        if self.level <= 0.0:
            self.cmd_publisher.publish(Twist())

        message = BatteryState()
        message.level = self.level
        message.is_charging = charging
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
