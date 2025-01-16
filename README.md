# Implementation of Path planning algorithms (A* and Dijkstra) in ROS2
Worked with a group of three COSI students to implement A* and Dijkstra path planning algorithms for a robot using ROS2.

1. **A star Algorithm Implementation**:
- Developed and tested the A* algorithm for pathfinding in a dynamic environment.
- Converted real-world coordinates to grid-based cost map coordinates for path calculations.
- Implemented a priority queue using cumulative costs and a heuristic for navigation.


2. **Dijkstra Algorithm Implementation**:

- Implemented the Dijkstra algorithm to compare performance with A*.
- Focused on cumulative costs for navigation without heuristics, suitable for short distances.
- Faced challenges with Obstacle, including frequent recoveries.

3. **ROS2 Configuration and Testing**:

- Edited ROS2 YAML parameter files for planner server configurations.
- Built and sourced ROS2 packages for testing and simulation.
- Used ros2 launch and ros2 run commands for running navigation simulations.
  
# Acknowledgements
This project is part of the Robotics and XR course 2024 at the University of Eastern Finland.
