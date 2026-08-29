#!/usr/bin/env python3

import math

import rclpy
from battery_interfaces.msg import BatteryState, RobotState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class DiagnosticsNode(Node):
    """Publish compact system health and mission statistics."""

    def __init__(self):
        super().__init__('diagnostics_node')
        self.level = 0.0
        self.charging = False
        self.state = 'STARTING'
        self.waypoint = -1
        self.distance = 0.0
        self.replans = 0
        self.avoidance_events = 0
        self.last_position = None
        self.last_state = None

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.summary_publisher = self.create_publisher(
            String, '/system_diagnostics', 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(RobotState, '/robot_state', self.state_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def battery_callback(self, message):
        self.level = message.level
        self.charging = message.is_charging

    def state_callback(self, message):
        self.state = message.state
        self.waypoint = message.current_waypoint_index
        if self.last_state != self.state:
            if self.state in ('PLANNING_PATH', 'NO_VALID_PATH'):
                self.replans += 1
            if self.state == 'AVOIDING_OBSTACLE':
                self.avoidance_events += 1
            self.last_state = self.state

    def odom_callback(self, message):
        position = message.pose.pose.position
        if self.last_position is not None:
            self.distance += math.hypot(
                position.x - self.last_position[0],
                position.y - self.last_position[1],
            )
        self.last_position = (position.x, position.y)

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'Charging Station AMR'
        status.hardware_id = 'turtlebot3_burger'
        if self.level <= 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Battery empty'
        elif self.level < 25.0:
            status.level = DiagnosticStatus.WARN
            status.message = 'Low battery'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'System operational'
        status.values = [
            KeyValue(key='battery_percent', value=f'{self.level:.1f}'),
            KeyValue(key='charging', value=str(self.charging)),
            KeyValue(key='robot_state', value=self.state),
            KeyValue(key='waypoint', value=str(self.waypoint)),
            KeyValue(key='distance_m', value=f'{self.distance:.2f}'),
            KeyValue(key='replans', value=str(self.replans)),
            KeyValue(key='obstacle_avoidance_events', value=str(self.avoidance_events)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diagnostics_publisher.publish(array)

        summary = String()
        summary.data = (
            f'battery={self.level:.1f}% charging={self.charging} '
            f'state={self.state} waypoint={self.waypoint} '
            f'distance={self.distance:.2f}m replans={self.replans} '
            f'avoidance_events={self.avoidance_events}'
        )
        self.summary_publisher.publish(summary)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
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
