#!/usr/bin/env python3

import math

import rclpy
from battery_interfaces.msg import BatteryState, RobotState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PatrolNode(Node):
    """Drive through waypoints and return to the charging pad when low."""

    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('low_battery_level', 25.0)
        self.declare_parameter('resume_battery_level', 80.0)
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('docking_distance', 0.30)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('obstacle_distance', 0.45)

        self.low_battery = float(self.get_parameter('low_battery_level').value)
        self.resume_battery = float(self.get_parameter('resume_battery_level').value)
        self.charger = (
            float(self.get_parameter('charger_x').value),
            float(self.get_parameter('charger_y').value),
        )
        self.docking_distance = float(self.get_parameter('docking_distance').value)
        self.tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.waypoints = [
            (0.8, 0.0), (0.8, 0.8), (0.0, 0.8), (-0.8, 0.8),
            (-0.8, 0.0), (-0.8, -0.8), (0.0, -0.8), (0.8, -0.8),
        ]
        self.x = None
        self.y = None
        self.yaw = None
        self.level = 100.0
        self.docked = False
        self.state = 'PATROLLING'
        self.resume_state = 'PATROLLING'
        self.waypoint_index = 0
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.cmd_publisher = self.create_publisher(Twist, '/raw_cmd_vel', 10)
        self.state_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Patrol node started with {len(self.waypoints)} waypoints')

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.docked = math.hypot(self.x - self.charger[0], self.y - self.charger[1]) <= self.docking_distance

    def battery_callback(self, message):
        self.level = message.level
        if self.level <= self.low_battery and self.state in ('PATROLLING', 'AVOIDING_OBSTACLE'):
            if self.state == 'AVOIDING_OBSTACLE':
                self.resume_state = 'RETURNING_TO_CHARGER'
            else:
                self.state = 'RETURNING_TO_CHARGER'
            self.get_logger().info(
                f'Battery low at {self.level:.1f}%, returning to charger')
        elif self.state == 'CHARGING' and self.level >= self.resume_battery:
            self.state = 'PATROLLING'
            self.get_logger().info(
                f'Battery recovered to {self.level:.1f}%, resuming patrol')

    def scan_callback(self, message):
        sectors = {'front': [], 'left': [], 'right': []}
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if not math.isfinite(distance):
                distance = message.range_max
            distance = max(message.range_min, min(message.range_max, distance))
            if abs(angle) <= 0.55:
                sectors['front'].append(distance)
            elif 0.55 < angle <= 1.57:
                sectors['left'].append(distance)
            elif -1.57 <= angle < -0.55:
                sectors['right'].append(distance)

        self.front_distance = min(sectors['front'], default=float('inf'))
        self.left_distance = min(sectors['left'], default=float('inf'))
        self.right_distance = min(sectors['right'], default=float('inf'))

    def target(self):
        if self.state in ('RETURNING_TO_CHARGER', 'CHARGING'):
            return self.charger
        return self.waypoints[self.waypoint_index]

    def control_loop(self):
        if self.x is None or self.yaw is None:
            return
        if self.state == 'RETURNING_TO_CHARGER' and self.docked:
            self.state = 'CHARGING'
        if self.state == 'CHARGING' and self.docked:
            self.publish_stop()
            self.publish_state()
            return

        if self.front_distance < self.obstacle_distance:
            if self.state != 'AVOIDING_OBSTACLE':
                self.resume_state = self.state
                self.state = 'AVOIDING_OBSTACLE'
            command = Twist()
            command.angular.z = 0.9 if self.left_distance > self.right_distance else -0.9
            self.cmd_publisher.publish(command)
            self.publish_state()
            return
        if self.state == 'AVOIDING_OBSTACLE':
            self.state = self.resume_state

        target_x, target_y = self.target()
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        if self.state == 'PATROLLING' and distance <= self.tolerance:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            target_x, target_y = self.target()
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))
        command = Twist()
        command.angular.z = max(-1.2, min(1.2, 2.0 * angle_error))
        if abs(angle_error) < 0.35:
            command.linear.x = min(self.max_linear_speed, 0.45 * distance)
            if self.front_distance < self.obstacle_distance + 0.25:
                command.linear.x *= 0.35
        self.cmd_publisher.publish(command)
        self.publish_state()

    def publish_stop(self):
        self.cmd_publisher.publish(Twist())

    def publish_state(self):
        message = RobotState()
        message.state = self.state
        message.current_waypoint_index = self.waypoint_index
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
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
