#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator
import heapq


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        # Initialize BasicNavigator to get the global costmap
        self.basic_navigator = BasicNavigator()

        # Create a service "create_plan"
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def create_plan_cb(self, request, response):
        # Get start and goal poses
        start_pose = request.start
        goal_pose = request.goal
        time_now = self.get_clock().now().to_msg()

        # Retrieve the global costmap
        global_costmap = self.basic_navigator.getGlobalCostmap()
        costmap = global_costmap.data
        width = global_costmap.info.width
        height = global_costmap.info.height
        resolution = global_costmap.info.resolution
        origin = global_costmap.info.origin

        # Convert the costmap data into a 2D array
        costmap_2d = [
            costmap[i * width:(i + 1) * width]
            for i in range(height)
        ]

        # Plan the path using A* algorithm
        response.path = a_star_planner(
            start_pose, goal_pose, costmap_2d, resolution, origin, time_now
        )
        return response


def a_star_planner(start, goal, costmap, resolution, origin, time_now):
    """
    A* algorithm to compute a path avoiding obstacles.

    Args:
        start (PoseStamped): Start pose.
        goal (PoseStamped): Goal pose.
        costmap (list of lists): 2D costmap grid.
        resolution (float): Resolution of the grid (meters per cell).
        origin (PoseStamped): Origin of the costmap (bottom-left corner).
        time_now (Time): Current ROS2 time.

    Returns:
        Path: Computed path as a `nav_msgs.msg.Path`.
    """
    path = Path()
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    # Convert start and goal to grid coordinates
    start_grid = (
        int((start.pose.position.x - origin.position.x) / resolution),
        int((start.pose.position.y - origin.position.y) / resolution),
    )
    goal_grid = (
        int((goal.pose.position.x - origin.position.x) / resolution),
        int((goal.pose.position.y - origin.position.y) / resolution),
    )

    def heuristic(a, b):
        """Calculate the Euclidean distance as the heuristic."""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    # Priority queue for A*
    open_set = []
    heapq.heappush(open_set, (0, start_grid))
    came_from = {}
    g_score = {start_grid: 0}

    while open_set:
        # Get the current node with the lowest cost
        _, current = heapq.heappop(open_set)

        # If goal is reached
        if current == goal_grid:
            # Reconstruct path
            while current in came_from:
                x, y = current
                pose = PoseStamped()
                pose.pose.position.x = x * resolution + origin.position.x
                pose.pose.position.y = y * resolution + origin.position.y
                pose.header.stamp = time_now
                pose.header.frame_id = goal.header.frame_id
                path.poses.append(pose)
                current = came_from[current]

            path.poses.reverse()
            return path

        # Get neighbors
        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            # Skip if neighbor is out of bounds or in obstacle
            if (neighbor[0] < 0 or neighbor[1] < 0 or
                    neighbor[0] >= len(costmap) or neighbor[1] >= len(costmap[0]) or
                    costmap[neighbor[1]][neighbor[0]] > 50):  # Threshold for obstacles
                continue

            # Calculate tentative g-score
            tentative_g_score = g_score[current] + resolution

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Update the path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal_grid)
                heapq.heappush(open_set, (f_score, neighbor))

    # If no path is found, return an empty path
    return path


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()

    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
