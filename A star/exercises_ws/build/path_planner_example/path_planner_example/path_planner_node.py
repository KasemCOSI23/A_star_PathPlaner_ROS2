#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator
import heapq

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        # Initialize attributes for the costmap
        self.global_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # Adjust this topic name if needed
            self.global_costmap_callback,
            10
        )

        # Create a service "create_plan"
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def global_costmap_callback(self, msg):
        """Callback to store the global costmap."""
        self.global_costmap = msg

    def create_plan_cb(self, request, response):
        # Ensure the global costmap is available
        if self.global_costmap is None:
            self.get_logger().error("Global costmap not available")
            return response

        costmap = self.global_costmap.data  # The 1D array of costmap data
        width = self.global_costmap.info.width  # Width of the costmap
        height = self.global_costmap.info.height  # Height of the costmap
        resolution = self.global_costmap.info.resolution  # Resolution of the costmap
        origin = self.global_costmap.info.origin  # Origin of the costmap

        # Plan the path using A* algorithm
        response.path = a_star_planner(
            request.start, request.goal, costmap, resolution, origin, self.get_clock().now().to_msg(), width, height
        )
        return response

def a_star_planner(start, goal, costmap, resolution, origin, time_now, width, height):
    """
    A* algorithm to compute a path avoiding obstacles.
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
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected grid
        ]

        for neighbor in neighbors:
            # Ensure that the neighbor is within bounds
            if (neighbor[0] < 0 or neighbor[1] < 0 or
                    neighbor[0] >= width or neighbor[1] >= height):
                continue

            # Calculate the index in the 1D costmap array
            index = neighbor[1] * width + neighbor[0]

            # Skip obstacles (threshold 50 as per costmap)
            if costmap[index] > 50:
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