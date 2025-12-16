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
        self.basic_navigator = BasicNavigator()
        # Create the service called by Nav2
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

        # --- PATH CACHING VARIABLES ---
        # Nav2 requests replanning frequently. To avoid recalculating the 
        # complex coverage path every second (which causes loops), 
        # we calculate it once and cache it.
        self.cached_path = None
        self.last_goal_pose = None
        
        # Tolerance: How close the new goal must be to the old one 
        # to be considered the "same task".
        self.goal_tolerance = 0.1 

    def create_plan_cb(self, request, response):
        """
        Callback function for the 'create_plan' service.
        Handles logic for new plans vs. replanning (pruning cached paths).
        """
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()
        
        # 1. CHECK IF THIS IS A NEW GOAL
        is_new_goal = True
        if self.last_goal_pose is not None:
            dist = math.hypot(
                goal_pose.pose.position.x - self.last_goal_pose.pose.position.x,
                goal_pose.pose.position.y - self.last_goal_pose.pose.position.y
            )
            # If the goal is very close to the previous one, it's the same task.
            if dist < self.goal_tolerance:
                is_new_goal = False

        # 2. IF NEW GOAL -> CALCULATE FULL COVERAGE PATH
        if is_new_goal:
            print(f"--- NEW GOAL RECEIVED! Calculating full coverage path... ---")
            global_costmap = self.basic_navigator.getGlobalCostmap()
            
            # Calculate the path and store it in cache (self.cached_path)
            # We use downsample_factor=6 to speed up calculation (coarse grid).
            full_path = create_coarse_coverage_plan(start_pose, goal_pose, time_now, global_costmap, downsample_factor=6)
            
            self.cached_path = full_path
            self.last_goal_pose = goal_pose
            
            response.path = full_path
            return response

        # 3. IF SAME GOAL -> USE CACHED PATH (PRUNED)
        else:
            if self.cached_path is None or len(self.cached_path.poses) == 0:
                return response 

            # Etsitään lähin kohta
            closest_idx = 0
            min_dist = float('inf')
            current_x = start_pose.pose.position.x
            current_y = start_pose.pose.position.y
            
            # Rajoitetaan hakua suorituskyvyn takia
            search_limit = min(len(self.cached_path.poses), 500) 
            
            for i in range(search_limit):
                px = self.cached_path.poses[i].pose.position.x
                py = self.cached_path.poses[i].pose.position.y
                d = math.hypot(px - current_x, py - current_y)
                if d < min_dist:
                    min_dist = d
                    closest_idx = i
            
            remaining_path = Path()
            remaining_path.header.frame_id = goal_pose.header.frame_id
            remaining_path.header.stamp = time_now

            # HARVENNUS (Downsampling): Otetaan vain joka 10. piste
            step = 10 
            subset_poses = self.cached_path.poses[closest_idx::step]

            # --- KORJAUS: PÄIVITETÄÄN AIKALEIMAT ---
            for old_pose in subset_poses:
                new_pose = PoseStamped()
                
                # Asetetaan NYKYINEN aika, jotta TF-muunnokset toimivat
                new_pose.header.frame_id = goal_pose.header.frame_id
                new_pose.header.stamp = time_now 
                
                # Kopioidaan sijainti vanhasta reitistä
                new_pose.pose.position.x = old_pose.pose.position.x
                new_pose.pose.position.y = old_pose.pose.position.y
                new_pose.pose.position.z = old_pose.pose.position.z
                new_pose.pose.orientation = old_pose.pose.orientation
                
                remaining_path.poses.append(new_pose)

            response.path = remaining_path
            return response

def create_coarse_coverage_plan(start, goal, time_now, costmap, downsample_factor=6):
    """
    Generates a "Coverage" style path using DFS (Depth-First Search) on a coarse grid.
    
    Args:
        downsample_factor (int): How many grid cells are merged into one. 
                                 6 means 30cm grid (if base is 5cm).
    """
    path = Path()
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    resolution = costmap.metadata.resolution
    origin_x = costmap.metadata.origin.position.x
    origin_y = costmap.metadata.origin.position.y
    width = costmap.metadata.size_x
    height = costmap.metadata.size_y
    data = costmap.data

    # --- 1. GRID CONVERSION & DOWNSAMPLING ---
    # Convert meters to grid indices
    s_x, s_y = world_to_grid(start.pose.position.x, start.pose.position.y, origin_x, origin_y, resolution)
    g_x, g_y = world_to_grid(goal.pose.position.x, goal.pose.position.y, origin_x, origin_y, resolution)

    # Convert to "Coarse Grid" (larger cells for performance)
    coarse_start = (s_x // downsample_factor, s_y // downsample_factor)
    coarse_goal = (g_x // downsample_factor, g_y // downsample_factor)
    
    c_width = width // downsample_factor
    c_height = height // downsample_factor

    # --- 2. DFS ALGORITHM (Depth-First Search) ---
    # We use a Stack to implement DFS. 
    stack = [coarse_start]
    came_from = {coarse_start: None}
    visited = {coarse_start}
    found_path = False

    print(f"Generating static coverage map ({c_width}x{c_height})...")

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

        # HEURISTIC SORTING:
        # Prioritize neighbors FURTHEST from the goal.
        # This forces the robot to explore the edges/corners before going to the goal.
        neighbors.sort(key=lambda n: abs(n[0] - coarse_goal[0]) + abs(n[1] - coarse_goal[1]))

        for nx, ny in neighbors:
            # Check bounds (Coarse Grid)
            if not (0 <= nx < c_width and 0 <= ny < c_height):
                continue
            
            if (nx, ny) in visited:
                continue

            # COLLISION CHECK
            # We map the center of the "Coarse Cell" back to the original Costmap
            real_x = nx * downsample_factor + (downsample_factor // 2)
            real_y = ny * downsample_factor + (downsample_factor // 2)
            
            # Check bounds (Real Map)
            if not (0 <= real_x < width and 0 <= real_y < height):
                continue

            idx = real_y * width + real_x
            
            # Threshold 50: Allows moving close to walls but avoids lethal obstacles (254).
            if data[idx] >= 50: 
                continue

            stack.append((nx, ny))
            visited.add((nx, ny))
            came_from[(nx, ny)] = current

    # --- 3. PATH RECONSTRUCTION ---
    if found_path:
        # Backtrack from goal to start
        curr = coarse_goal
        coarse_path = []
        while curr != coarse_start:
            coarse_path.append(curr)
            curr = came_from.get(curr)
            if curr is None: break
        coarse_path.append(coarse_start)
        coarse_path.reverse() # Reverse to get Start -> Goal

        print(f"Coverage path generated: {len(coarse_path)} steps.")

        # Convert back to World Coordinates and create PoseStamped messages
        for (cx, cy) in coarse_path:
            pose = PoseStamped()
            
            # Map coarse grid back to world meters (center of the cell)
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

# --- HELPER FUNCTIONS ---

def world_to_grid(wx, wy, ox, oy, resolution):
    """ Converts meters to grid index """
    gx = int((wx - ox) / resolution)
    gy = int((wy - oy) / resolution)
    return (gx, gy)

def grid_to_world(gx, gy, ox, oy, resolution):
    """ Converts grid index to meters (center of cell) """
    wx = (gx * resolution) + ox + (resolution / 2)
    wy = (gy * resolution) + oy + (resolution / 2)
    return (wx, wy)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()