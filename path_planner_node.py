#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        # Uncommented to get Global Costmap in create_plan_cb
        self.basic_navigator = BasicNavigator()

        # Creating a new service "create_plan", which is called by our Nav2 C++ planner plugin
        # to receive a plan from us.
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

        # Variables for caching the path to handle Nav2 replanning loops
        self.cached_path = None
        self.last_goal_pose = None
        self.goal_tolerance = 0.1  # Distance to consider a goal "new"

    def create_plan_cb(self, request, response):
        # Getting all the information to plan the path
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()
        
        # Uncommented to get Global CostMap
        global_costmap = self.basic_navigator.getGlobalCostmap()

        print("----")
        print(f"Starting pose: ({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f})")
        print(f"Goal pose: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")

        # --- LOGIC TO HANDLE REPLANNING ---
        
        # 1. Check if this is a new goal or just a replanning request for the same goal
        is_new_goal = True
        if self.last_goal_pose is not None:
            dist = math.hypot(
                goal_pose.pose.position.x - self.last_goal_pose.pose.position.x,
                goal_pose.pose.position.y - self.last_goal_pose.pose.position.y
            )
            if dist < self.goal_tolerance:
                is_new_goal = False

        # 2. If it is a NEW goal, calculate the full coverage path from scratch
        if is_new_goal:
            print("--- NEW GOAL: Calculating full coverage path... ---")
            
            # Use the custom coverage planner (replacing straight line planner)
            # We use a downsample factor of 6 (30cm grids) to improve performance
            full_path = create_coarse_coverage_plan(start_pose, goal_pose, time_now, global_costmap, downsample_factor=6)
            
            # Cache the result
            self.cached_path = full_path
            self.last_goal_pose = goal_pose
            
            response.path = full_path
            return response

        # 3. If it is the SAME goal, prune the cached path (remove already visited points)
        else:
            # Logic to find where the robot is currently on the path
            if self.cached_path is None or len(self.cached_path.poses) == 0:
                return response 

            # Find the index of the point closest to the current robot position
            closest_idx = 0
            min_dist = float('inf')
            
            current_x = start_pose.pose.position.x
            current_y = start_pose.pose.position.y

            # Search limit optimization
            search_limit = min(len(self.cached_path.poses), 500) 
            
            for i in range(search_limit):
                px = self.cached_path.poses[i].pose.position.x
                py = self.cached_path.poses[i].pose.position.y
                d = math.hypot(px - current_x, py - current_y)
                
                if d < min_dist:
                    min_dist = d
                    closest_idx = i
            
            # Create a new path containing only the remaining points
            remaining_path = Path()
            remaining_path.header.frame_id = goal_pose.header.frame_id
            remaining_path.header.stamp = time_now
            
            # Append points starting from the current robot position
            for old_pose in self.cached_path.poses[closest_idx:]:
                new_pose = PoseStamped()
                new_pose.pose = old_pose.pose
                new_pose.header.frame_id = goal_pose.header.frame_id
                new_pose.header.stamp = time_now
                remaining_path.poses.append(new_pose)

            response.path = remaining_path
            return response


def create_coarse_coverage_plan(start, goal, time_now, costmap, downsample_factor=6):
    """ 
    Creates a 'Coverage' style path using DFS (Depth-First Search) on a coarse grid.
    Instead of finding the shortest path, it attempts to cover the area.
    Using a coarse grid (downsampling) prevents performance timeouts.
    """
    path = Path()
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    # Extract metadata from the costmap
    resolution = costmap.metadata.resolution
    origin_x = costmap.metadata.origin.position.x
    origin_y = costmap.metadata.origin.position.y
    width = costmap.metadata.size_x
    height = costmap.metadata.size_y
    data = costmap.data

    # Convert World coordinates to Grid coordinates
    s_x, s_y = world_to_grid(start.pose.position.x, start.pose.position.y, origin_x, origin_y, resolution)
    g_x, g_y = world_to_grid(goal.pose.position.x, goal.pose.position.y, origin_x, origin_y, resolution)

    # Convert to Coarse Grid (Downsampling)
    coarse_start = (s_x // downsample_factor, s_y // downsample_factor)
    coarse_goal = (g_x // downsample_factor, g_y // downsample_factor)
    
    c_width = width // downsample_factor
    c_height = height // downsample_factor

    # DFS Algorithm setup
    stack = [coarse_start]
    came_from = {coarse_start: None}
    visited = {coarse_start}
    found_path = False

    print(f"Generating coverage path on coarse grid ({c_width}x{c_height})...")

    while stack:
        current = stack.pop()

        if current == coarse_goal:
            found_path = True
            break

        neighbors = [
            (current[0] + 1, current[1]),
            (current[0] - 1, current[1]),
            (current[0], current[1] + 1),
            (current[0], current[1] - 1)
        ]

        # Sort neighbors: Put the one CLOSEST to goal at the bottom of the stack,
        # and the FARTHEST at the top. This encourages the robot to take the long way.
        neighbors.sort(key=lambda n: abs(n[0] - coarse_goal[0]) + abs(n[1] - coarse_goal[1]))

        for nx, ny in neighbors:
            # Check boundaries of coarse grid
            if not (0 <= nx < c_width and 0 <= ny < c_height):
                continue
            
            if (nx, ny) in visited:
                continue

            # Check for Obstacles in the real costmap
            # We check the center pixel of the "coarse block"
            real_x = nx * downsample_factor + (downsample_factor // 2)
            real_y = ny * downsample_factor + (downsample_factor // 2)
            
            # Boundary check for real grid
            if not (0 <= real_x < width and 0 <= real_y < height):
                continue

            idx = real_y * width + real_x
            
            # Threshold for obstacles (0 = free, 100-254 = obstacle/lethal)
            # Using a high threshold (230) allows moving close to walls
            if data[idx] >= 230: 
                continue

            stack.append((nx, ny))
            visited.add((nx, ny))
            came_from[(nx, ny)] = current

    # Reconstruct path if found
    if found_path:
        curr = coarse_goal
        coarse_path = []
        while curr != coarse_start:
            coarse_path.append(curr)
            curr = came_from.get(curr)
            if curr is None: break
        coarse_path.append(coarse_start)
        coarse_path.reverse()

        print(f"Coverage path generated with {len(coarse_path)} coarse steps.")

        # Convert back to PoseStamped messages
        for (cx, cy) in coarse_path:
            pose = PoseStamped()
            # Convert coarse grid back to world coordinates
            grid_x = cx * downsample_factor + (downsample_factor / 2)
            grid_y = cy * downsample_factor + (downsample_factor / 2)
            wx, wy = grid_to_world(grid_x, grid_y, origin_x, origin_y, resolution)
            
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.header = path.header
            path.poses.append(pose)
    else:
        print("No path found.")

    return path


def world_to_grid(wx, wy, ox, oy, resolution):
    """
    Helper to convert World coordinates (meters) to Grid coordinates (indices).
    """
    gx = int((wx - ox) / resolution)
    gy = int((wy - oy) / resolution)
    return (gx, gy)


def grid_to_world(gx, gy, ox, oy, resolution):
    """
    Helper to convert Grid coordinates (indices) to World coordinates (meters).
    """
    wx = (gx * resolution) + ox + (resolution / 2)
    wy = (gy * resolution) + oy + (resolution / 2)
    return (wx, wy)


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
