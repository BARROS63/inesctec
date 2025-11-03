import random
import time
from typing import List, Tuple, Dict
import numpy as np
import heapq
from math import sqrt, cos, radians
import matplotlib.pyplot as plt

# --- Constants ---
CAGE_WIDTH = 8  # meters (x-axis)
CAGE_LENGTH = 4  # meters (y-axis)
RESOLUTION = 0.1  # meters per grid cell
UPDATE_INTERVAL = 10  # seconds

# --- A* Components ---

def create_node(position: Tuple[int, int], g: float = float('inf'), 
                h: float = 0.0, parent: Dict = None) -> Dict:
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }

def calculate_heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    x1, y1 = pos1
    x2, y2 = pos2
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]:
    x, y = position
    rows, cols = grid.shape
    possible_moves = [
        (x+1, y), (x-1, y),
        (x, y+1), (x, y-1),
        (x+1, y+1), (x-1, y-1),
        (x+1, y-1), (x-1, y+1)
    ]
    return [
        (nx, ny) for nx, ny in possible_moves
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0
    ]

def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]:
    path = []
    current = goal_node
    while current is not None:
        path.append(current['position'])
        current = current['parent']
    return path[::-1]

def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    start_node = create_node(position=start, g=0, h=calculate_heuristic(start, goal))
    open_list = [(start_node['f'], start)]
    open_dict = {start: start_node}
    closed_set = set()
    
    while open_list:
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]
        
        if current_pos == goal:
            return reconstruct_path(current_node)
        
        closed_set.add(current_pos)
        
        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            if neighbor_pos in closed_set:
                continue
            tentative_g = current_node['g'] + calculate_heuristic(current_pos, neighbor_pos)
            if neighbor_pos not in open_dict or tentative_g < open_dict[neighbor_pos]['g']:
                neighbor = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic(neighbor_pos, goal),
                    parent=current_node
                )
                open_dict[neighbor_pos] = neighbor
                heapq.heappush(open_list, (neighbor['f'], neighbor_pos))
    return []

# --- Conversion Functions ---

def meters_to_grid(meters: float) -> int:
    return int(round(meters / RESOLUTION))

def grid_to_meters(grid: int) -> float:
    return grid * RESOLUTION

def generate_random_obstacle(grid: np.ndarray) -> Tuple[int, int]:
    """Generate random obstacle within cage limits"""
    max_x = meters_to_grid(CAGE_WIDTH)
    max_y = meters_to_grid(CAGE_LENGTH)
    return (random.randint(1, max_x-1), random.randint(1, max_y-1))

def place_obstacle(grid: np.ndarray, position: Tuple[int, int], size: int = 2):
    """Place square obstacle on grid"""
    x, y = position
    for dx in range(-size//2, size//2 + 1):
        for dy in range(-size//2, size//2 + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                grid[nx, ny] = 1

# --- Visualization ---

def visualize_path(grid: np.ndarray, path: List[Tuple[int, int]], 
                  start: Tuple[int, int], goal: Tuple[int, int]):
    plt.clf()
    plt.figure(figsize=(10, 6))
    plt.imshow(grid.T, cmap='gray_r', origin='lower', 
              extent=[0, grid_to_meters(grid.shape[0]), 
                      0, grid_to_meters(grid.shape[1])])
    
    if path:
        path_np = np.array(path)
        plt.plot(grid_to_meters(path_np[:, 0]), 
                grid_to_meters(path_np[:, 1]), 
                'b-', linewidth=2, label='Path')
    
    plt.scatter(grid_to_meters(start[0]), grid_to_meters(start[1]), 
               c='green', s=100, label='Start', edgecolors='black')
    plt.scatter(grid_to_meters(goal[0]), grid_to_meters(goal[1]), 
               c='red', s=100, label='Goal', edgecolors='black')
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title(f"A* Pathfinding (Obstacle at {grid_to_meters(obstacle_pos[0]):.1f}, {grid_to_meters(obstacle_pos[1]):.1f})")
    plt.legend()
    plt.grid(True)
    plt.pause(0.1)

# --- Main Execution ---

def main():
    global obstacle_pos
    
    # Grid dimensions
    grid_width = meters_to_grid(CAGE_WIDTH)
    grid_height = meters_to_grid(CAGE_LENGTH)
    
    # Fixed start and goal positions
    start_grid = (meters_to_grid(0.5), meters_to_grid(0.5))  # Bottom-left corner
    goal_grid = (meters_to_grid(CAGE_WIDTH-0.5), meters_to_grid(CAGE_LENGTH-0.5))  # Top-right corner
    
    plt.ion()  # Interactive mode
    
    for i in range(5):  # Run for 5 iterations (50 seconds total)
        # Create empty grid
        grid = np.zeros((grid_width, grid_height))
        
        # Generate and place random obstacle
        obstacle_pos = generate_random_obstacle(grid)
        place_obstacle(grid, obstacle_pos)
        
        print(f"\n--- Iteration {i+1} ---")
        print(f"Obstacle at grid position: {obstacle_pos}")
        print(f"Physical position: {grid_to_meters(obstacle_pos[0]):.1f}m, {grid_to_meters(obstacle_pos[1]):.1f}m")
        
        # Find path
        path = find_path(grid, start_grid, goal_grid)
        
        if not path:
            print("No path found!")
        else:
            print(f"Path found with {len(path)} waypoints")
        
        # Visualization
        visualize_path(grid, path, start_grid, goal_grid)
        
        # Wait for next iteration
        time.sleep(UPDATE_INTERVAL)
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()