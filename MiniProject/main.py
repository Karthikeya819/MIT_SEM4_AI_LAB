import numpy as np
from scipy.ndimage import gaussian_filter
import cv2
import yaml
from queue import PriorityQueue
import os
import glob


def reset_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)
    else:
        delfiles = glob.glob(path+"/*")
        for f in delfiles:
            if os.path.isfile(f):
                os.remove(f)

def compute_gradient(depth_grid):
    grad_y, grad_x = np.gradient(depth_grid)
    return np.hypot(grad_x, grad_y)

def compute_potential_field(depth_grid, goal, obstacle_weight=2.0, goal_weight=2.0, 
                          sigma_obstacle=1, sigma_goal=3, slope_weight=2.0,
                          max_slope_threshold=0.5):
    depth_min, depth_max = depth_grid.min(), depth_grid.max()
    depth_normalized = (depth_grid - depth_min) / (depth_max - depth_min + 1e-6)
    
    # Stronger slope penalties
    slope_field = compute_gradient(depth_grid)
    slope_field = np.clip(slope_field, 0, max_slope_threshold)
    slope_field_normalized = slope_field / max_slope_threshold
    
    # Enhanced obstacle field with stronger penalties
    obstacle_field = gaussian_filter(1 - depth_normalized, sigma=sigma_obstacle)
    slope_penalty = gaussian_filter(slope_field_normalized, sigma=sigma_obstacle)
    
    # Increased weights for obstacles and slopes
    potential = (obstacle_weight * obstacle_field + 
                slope_weight * slope_penalty)
    
    # Reduced goal attraction for more flexibility in path selection
    goal_field = np.zeros_like(depth_grid)
    goal_field[goal] = 1
    goal_field = gaussian_filter(goal_field, sigma=sigma_goal)
    potential -= goal_weight * goal_field
    
    # Stronger penalties for steep areas
    potential[slope_field_normalized > 0.7] *= 3.0
    
    return potential

def is_valid_move(depth_grid, pos):
    """Simple bounds check"""
    rows, cols = depth_grid.shape
    return (0 <= pos[0] < rows and 0 <= pos[1] < cols)

def heuristic(depth_grid,node, goal):
    dx, dy = node[0] - goal[0], node[1] - goal[1]
    depth_diff = abs(depth_grid[node] - depth_grid[goal])
    return np.hypot(dx, dy) + 5.0 * depth_diff

def a_star_with_potential(depth_grid, start, goal, max_iterations=50000000):
    if not (is_valid_move(depth_grid, start) and is_valid_move(depth_grid, goal)):
        return None
    
    print(f"\nStarting A* search from {start} to {goal}")
    print(f"Depth grid shape: {depth_grid.shape}")
    print(f"Depth grid range: {depth_grid.min():.3f} to {depth_grid.max():.3f}")

    # cv2.imshow("depth_grid",depth_grid)
    # cv2.waitKey(0)
    
    potential_field = compute_potential_field(depth_grid, goal)
    # cv2.imshow("potential field",potential_field)
    # cv2.waitKey(0)
    slope_field = compute_gradient(depth_grid)
    # cv2.imshow("slope_field",slope_field)
    # cv2.waitKey(0)

    
    rows, cols = depth_grid.shape
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    movements = [(0, 1), (1, 0), (0, -1), (-1, 0), 
                (1, 1), (1, -1), (-1, 1), (-1, -1)]
    movement_costs = [np.sqrt(dx*dx + dy*dy) * (1.2 if dx and dy else 1.0) 
                     for dx, dy in movements]
    
    explored = 0
    iterations = 0
    while not open_set.empty() and iterations < max_iterations:
        iterations += 1
        current_cost, current = open_set.get()
        explored += 1
        
        if iterations % 1000 == 0:
            print(f"Iteration {iterations}, Explored {explored} nodes")
        
        if current == goal:
            print(f"Goal reached after {iterations} iterations!")
            break
            
        current_row, current_col = current
        
        for (dx, dy), base_cost in zip(movements, movement_costs):
            new_row, new_col = current_row + dx, current_col + dy
            
            if not (0 <= new_row < rows and 0 <= new_col < cols):
                continue
                
            neighbor = (new_row, new_col)
            
            # Enhanced cost calculations prioritizing terrain quality
            slope_val = slope_field[neighbor]
            depth_val = depth_grid[neighbor]
            potential_val = potential_field[neighbor]

            if slope_val > 0.7 or potential_val > 0.9:  # Skip invalid nodes early
                continue

            depth_change_penalty = abs(depth_val - depth_grid[current])
            
            # Increased weights for terrain factors
            terrain_cost = (
                30.0 * slope_val +  # slope penalty 30
                0.01 * (1.0 - depth_val) +  # depth penalty
                0.01 * max(0, potential_val) +  # potential field penalty
                15 * depth_change_penalty   # New penalty for depth variation 15
            )
            
            total_cost = base_cost * (1.0 + terrain_cost)
            depth_diff = abs(depth_grid[new_row, new_col] - depth_grid[current_row, current_col])
            new_cost = cost_so_far[current] + base_cost * (1.0 + terrain_cost + 10.0 * depth_diff)
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + 0.5 * heuristic(depth_grid, neighbor, goal)
                open_set.put((priority, neighbor))
                came_from[neighbor] = current
    
    if goal not in came_from:
        print(f"Failed to find path after {iterations} iterations")
        return None
        
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    
    print(f"Found path with {len(path)} points")
    return path

def process_image(rgb_path, depth_path, proc_depth_path, coords, output_path):
    print(f"\nProcessing image:")
    print(f"RGB path: {rgb_path}")
    print(f"Depth path: {depth_path}")
    print(f"Coordinates: {coords}")
    
    rgb = cv2.imread(rgb_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_GRAYSCALE)
    proc_depth = cv2.imread(proc_depth_path)

    # print(proc_depth.shape)
    # print(depth.shape)
    
    if rgb is None or depth is None:
        print("Failed to load images")
        return
        
    print(f"RGB shape: {rgb.shape}")
    print(f"Depth shape: {depth.shape}")
    
    depth_normalized = (depth.astype(float) - np.min(depth)) / (np.max(depth) - np.min(depth))
    print(f"Depth range: {depth_normalized.min():.3f} to {depth_normalized.max():.3f}")
    # cv2.imshow("rgb", depth_normalized)
    # cv2.waitKey(0)
    
    complete_path = []
    if len(coords) > 1:
        print("dk")
        for i in range(len(coords) - 1):
            # Swap x,y to row,col for internal processing
            start = (coords[i][1], coords[i][0])
            goal = (coords[i + 1][1], coords[i + 1][0])
            
            print(f"\nPlanning segment {i+1}/{len(coords)-1}")
            print(f"Start: {start}")
            print(f"Goal: {goal}")
            
            path_segment = a_star_with_potential(depth_normalized, start, goal)
            
            if path_segment:
                if not complete_path:
                    complete_path.extend(path_segment)
                else:
                    complete_path.extend(path_segment[1:])
                print(f"Added segment with {len(path_segment)} points")
            else:
                print("Failed to find path for this segment")
    
    if complete_path:
        print(f"\nVisualization:")
        print(f"Total path length: {len(complete_path)}")
        visualize_path(rgb, proc_depth, complete_path, output_path)
    else:
        print("\nNo path to visualize")
        cv2.imwrite(output_path, rgb)

def visualize_path(image, proc_depth, path, f_name):
    output_img = image.copy()
    if path:
        path_points = np.array(path, dtype=np.int32)
        for i in range(len(path_points) - 1):
            pt1 = (path_points[i][1], path_points[i][0])  # Swap x,y for OpenCV
            pt2 = (path_points[i + 1][1], path_points[i + 1][0])
            cv2.line(output_img, pt1, pt2, (0, 0, 255), 2)
        for i, point in enumerate(path_points):
            color = (0, 255, 0) if i == 0 else (255, 0, 0) if i == len(path_points)-1 else (0, 0, 255)
            cv2.circle(output_img, (point[1], point[0]), 5, color, -1)
    
    cv2.imwrite(f_name, output_img)
    print(f"Saved path visualization to {f_name}")

def main():
    file_no = 7
    rgb_path = f"E:/MIT_SEM4_AI_LAB/MiniProject/Samples/{file_no}/image_rgb.jpg"
    depth_path = f"E:/MIT_SEM4_AI_LAB/MiniProject/Samples/{file_no}/image_depth.webp"
    proc_depth_path = f"E:/MIT_SEM4_AI_LAB/MiniProject/Samples/{file_no}/image_depth.webp"
    output_path = f"E:/MIT_SEM4_AI_LAB/MiniProject/Samples/{file_no}/image_rgb_proc.webp"
    points = []
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            if len(points) == 2:
                cv2.destroyAllWindows()
    img = cv2.imread(rgb_path)
    cv2.imshow("Select Start, Goal", img)
    cv2.setMouseCallback("Select Start, Goal", click_event)
    cv2.waitKey(0)
    process_image(rgb_path, depth_path, proc_depth_path, points, output_path)

if __name__ == "__main__":
    main()