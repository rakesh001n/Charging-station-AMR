import unittest
from types import SimpleNamespace

from battery_patrol_robot.battery_node import BatteryNode
from battery_patrol_robot.navigation_node import NavigationNode


class NavigationPlannerTest(unittest.TestCase):
    def setUp(self):
        self.node = NavigationNode.__new__(NavigationNode)
        self.node.resolution = 0.10
        self.node.bounds = (-2.4, 2.4, -2.4, 2.4)
        self.node.obstacle_inflation = 0.25

    def test_blue_obstacle_is_not_a_valid_goal(self):
        self.assertEqual(self.node.plan((0.8, 0.0), (0.8, 0.8)), [])

    def test_planner_finds_route_around_blue_obstacle(self):
        route = self.node.plan((0.8, 0.0), (0.0, 0.8))
        self.assertTrue(route)
        self.assertTrue(all(not self.node.blocked(point) for point in route))


class BatterySafetyTest(unittest.TestCase):
    def test_empty_battery_forwards_zero_velocity(self):
        node = BatteryNode.__new__(BatteryNode)
        node.level = 0.0
        published = []
        node.cmd_publisher = SimpleNamespace(publish=published.append)
        node.cmd_callback(SimpleNamespace())
        self.assertEqual(published[0].linear.x, 0.0)
        self.assertEqual(published[0].angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()
