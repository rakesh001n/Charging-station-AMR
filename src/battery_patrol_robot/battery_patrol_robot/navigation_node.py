#!/usr/bin/env python3

import heapq
import math

import rclpy
from battery_interfaces.msg import BatteryState, RobotState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class NavigationNode(Node):
    """A* global planner with laser avoidance and mission supervision."""

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('low_battery_level', 25.0)
        self.declare_parameter('resume_battery_level', 80.0)
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)
        self.declare_parameter('docking_distance', 0.30)
        self.declare_parameter('obstacle_distance', 0.45)
        self.declare_parameter('critical_obstacle_distance', 0.22)
        self.declare_parameter('obstacle_inflation', 0.20)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.35)

        self.low_battery = float(self.get_parameter('low_battery_level').value)
        self.resume_battery = float(self.get_parameter('resume_battery_level').value)
        self.charger = (float(self.get_parameter('charger_x').value),
                        float(self.get_parameter('charger_y').value))
        self.docking_distance = float(self.get_parameter('docking_distance').value)
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.critical_distance = float(self.get_parameter('critical_obstacle_distance').value)
        self.obstacle_inflation = float(self.get_parameter('obstacle_inflation').value)
        self.tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.resolution = 0.10
        self.bounds = (-2.4, 2.4, -2.4, 2.4)
        self.waypoints = [(0.8, 0.0), (0.8, 0.8), (0.0, 0.8), (-0.8, 0.8),
                          (-0.8, 0.0), (-0.8, -0.8), (0.0, -0.8), (0.8, -0.8)]

        self.x = self.y = self.yaw = None
        self.level = 100.0
        self.docked = False
        self.emergency_stop = False
        self.paused = False
        self.state = 'PLANNING_PATH'
        self.resume_state = 'PATROLLING'
        self.waypoint_index = 0
        self.route = []
        self.route_target = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.avoid_turn_direction = None
        self.last_plan_time = self.get_clock().now()

        self.cmd_publisher = self.create_publisher(Twist, '/raw_cmd_vel', 10)
        self.state_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.stop_callback, 10)
        self.create_service(Trigger, '/pause_mission', self.pause_callback)
        self.create_service(Trigger, '/resume_mission', self.resume_callback)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation node started with A* planning')

    def grid(self, point):
        return (round((point[0] - self.bounds[0]) / self.resolution),
                round((point[1] - self.bounds[2]) / self.resolution))

    def world(self, cell):
        return (self.bounds[0] + cell[0] * self.resolution,
                self.bounds[2] + cell[1] * self.resolution)

    def blocked(self, point):
        x, y = point
        if not (self.bounds[0] <= x <= self.bounds[1] and self.bounds[2] <= y <= self.bounds[3]):
            return True
        margin = self.obstacle_inflation
        # Blue box: 0.7 m square centered at (1.2, 0.8), inflated for the robot.
        if 0.85 - margin <= x <= 1.55 + margin and 0.45 - margin <= y <= 1.15 + margin:
            return True
        # Orange box, conservatively inflated around its rotated footprint.
        if -1.8 - margin <= x <= -0.8 + margin and -1.35 - margin <= y <= -0.45 + margin:
            return True
        return math.hypot(x, y + 1.8) <= 0.35 + margin

    def nearest_free_cell(self, cell):
        if not self.blocked(self.world(cell)):
            return cell
        for radius in range(1, 10):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if not self.blocked(self.world(candidate)):
                        return candidate
        return None

    def plan(self, start, goal):
        start_cell = self.grid(start)
        goal_cell = self.grid(goal)
        start_cell = self.nearest_free_cell(start_cell)
        if self.blocked(self.world(goal_cell)):
            return []
        if start_cell is None:
            return []
        queue = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal_cell:
                path = []
                while current is not None:
                    path.append(self.world(current))
                    current = came_from[current]
                return list(reversed(path))[1:]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                neighbor = (current[0] + dx, current[1] + dy)
                if self.blocked(self.world(neighbor)):
                    continue
                step = math.sqrt(2.0) if dx and dy else 1.0
                new_cost = cost[current] + step
                if new_cost < cost.get(neighbor, float('inf')):
                    cost[neighbor] = new_cost
                    heuristic = math.hypot(goal_cell[0] - neighbor[0], goal_cell[1] - neighbor[1])
                    heapq.heappush(queue, (new_cost + heuristic, neighbor))
                    came_from[neighbor] = current
        return []

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.x, self.y = position.x, position.y
        sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.docked = math.hypot(self.x - self.charger[0], self.y - self.charger[1]) <= self.docking_distance

    def battery_callback(self, message):
        self.level = message.level
        if self.level <= self.low_battery and self.state in (
                'PATROLLING', 'PLANNING_PATH', 'NAVIGATING',
                'AVOIDING_OBSTACLE', 'NO_VALID_PATH'):
            self.state = 'RETURNING_TO_CHARGER'
            self.route = []
            self.get_logger().info(
                f'Low battery {self.level:.1f}%: planning return to charger')
        elif self.state == 'CHARGING' and self.level >= self.resume_battery:
            self.state = 'PATROLLING'
            self.route = []
            self.get_logger().info(
                f'Battery {self.level:.1f}%: resuming patrol')

    def stop_callback(self, message):
        self.emergency_stop = message.data
        if self.emergency_stop:
            self.state = 'EMERGENCY_STOP'
            self.publish_stop()
        elif self.state == 'EMERGENCY_STOP':
            self.state = 'PATROLLING'
            self.route = []

    def pause_callback(self, request, response):
        self.paused = True
        self.state = 'PAUSED'
        self.publish_stop()
        response.success = True
        response.message = 'Mission paused'
        return response

    def resume_callback(self, request, response):
        if self.emergency_stop:
            response.success = False
            response.message = 'Cannot resume while emergency stop is active'
            return response
        self.paused = False
        self.state = 'RETURNING_TO_CHARGER' if self.level <= self.low_battery else 'PLANNING_PATH'
        self.route = []
        response.success = True
        response.message = 'Mission resumed'
        return response

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

    def goal(self):
        return self.charger if self.state == 'RETURNING_TO_CHARGER' else self.waypoints[self.waypoint_index]

    def replan(self):
        if self.x is None:
            return False
        goals = [(self.goal(), self.waypoint_index)]
        if self.state != 'RETURNING_TO_CHARGER':
            goals += [
                (self.waypoints[(self.waypoint_index + offset) % len(self.waypoints)],
                 (self.waypoint_index + offset) % len(self.waypoints))
                for offset in range(1, len(self.waypoints))
            ]
        for goal, goal_index in goals:
            route = self.plan((self.x, self.y), goal)
            if route:
                if self.state != 'RETURNING_TO_CHARGER' and goal_index != self.waypoint_index:
                    self.get_logger().warning(
                        f'Waypoint {self.waypoint_index} blocked; '
                        f'redirecting to waypoint {goal_index}')
                    self.waypoint_index = goal_index
                self.route = route
                self.route_target = self.goal()
                self.publish_path(route)
                self.state = 'NAVIGATING' if self.state == 'PLANNING_PATH' else self.state
                return True
        self.route = []
        self.state = 'NO_VALID_PATH'
        self.get_logger().warning('No valid path found; will retry')
        return False

    def control_loop(self):
        if self.x is None or self.yaw is None or self.emergency_stop or self.paused:
            self.publish_stop()
            self.publish_state()
            return
        if self.state == 'RETURNING_TO_CHARGER' and self.docked:
            self.state = 'CHARGING'
            self.route = []
        if self.state == 'CHARGING':
            self.publish_stop()
            self.publish_state()
            return
        if self.state == 'NO_VALID_PATH':
            self.publish_stop()
            if (self.get_clock().now() - self.last_plan_time).nanoseconds > 2e9:
                self.last_plan_time = self.get_clock().now()
                self.state = 'PLANNING_PATH'
            self.publish_state()
            return
        if self.front_distance < self.critical_distance:
            if self.avoid_turn_direction is None:
                self.avoid_turn_direction = 1.0 if self.left_distance >= self.right_distance else -1.0
            recovery = Twist()
            recovery.angular.z = self.avoid_turn_direction * 0.7
            self.cmd_publisher.publish(recovery)
            self.publish_state('AVOIDING_OBSTACLE')
            self.route = []
            return
        if self.front_distance < self.obstacle_distance:
            if self.avoid_turn_direction is None:
                self.avoid_turn_direction = 1.0 if self.left_distance >= self.right_distance else -1.0
            command = Twist()
            command.angular.z = self.avoid_turn_direction * 0.9
            self.cmd_publisher.publish(command)
            self.publish_state('AVOIDING_OBSTACLE')
            self.route = []
            return
        self.avoid_turn_direction = None
        if not self.route or self.route_target != self.goal():
            if self.state != 'RETURNING_TO_CHARGER':
                self.state = 'PLANNING_PATH'
            if not self.replan():
                self.publish_state()
                return
        target_x, target_y = self.route[0]
        if math.hypot(target_x - self.x, target_y - self.y) < self.tolerance:
            self.route.pop(0)
            if not self.route:
                if self.state == 'RETURNING_TO_CHARGER':
                    self.state = 'CHARGING' if self.docked else 'PLANNING_PATH'
                else:
                    self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
                    self.state = 'PLANNING_PATH'
                self.publish_stop()
                self.publish_state()
                return
            target_x, target_y = self.route[0]
        angle = math.atan2(target_y - self.y, target_x - self.x)
        error = math.atan2(math.sin(angle - self.yaw), math.cos(angle - self.yaw))
        command = Twist()
        command.angular.z = max(-1.2, min(1.2, 2.0 * error))
        if abs(error) < 0.35:
            command.linear.x = min(
                self.max_linear_speed,
                0.45 * math.hypot(target_x - self.x, target_y - self.y),
            )
        self.cmd_publisher.publish(command)
        self.publish_state()

    def publish_stop(self):
        self.cmd_publisher.publish(Twist())

    def publish_path(self, points):
        message = Path()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def publish_state(self, override=None):
        message = RobotState()
        message.state = override or self.state
        message.current_waypoint_index = self.waypoint_index
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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
