from typing import List, Tuple, Dict
import numpy as np
import heapq
from math import sqrt, cos, radians
import matplotlib.pyplot as plt

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

def meters_to_lat(meters: float) -> float:
    return meters / 111320

def meters_to_lon(meters: float, latitude: float) -> float:
    return meters / (111320 * cos(radians(latitude)))

def latlon_to_grid(start_lat: float, start_lon: float, lat: float, lon: float, resolution: float) -> Tuple[int, int]:
    dy = (lat - start_lat) * 111320
    dx = (lon - start_lon) * (111320 * cos(radians(start_lat)))
    return int(round(dx / resolution)), int(round(dy / resolution))

def grid_to_latlon(start_lat: float, start_lon: float, grid_point: Tuple[int, int], resolution: float) -> Tuple[float, float]:
    dx = grid_point[0] * resolution
    dy = grid_point[1] * resolution
    lat = start_lat + (dy / 111320)
    lon = start_lon + (dx / (111320 * cos(radians(start_lat))))
    return lat, lon

# --- Visualisation ---

def visualize_path(grid: np.ndarray, path: List[Tuple[int, int]], start: Tuple[int, int], goal: Tuple[int, int]):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='gray_r', origin='lower')
    
    if path:
        path_np = np.array(path)
        plt.plot(path_np[:, 1], path_np[:, 0], 'b-', linewidth=2, label='Path')
    
    plt.scatter(start[1], start[0], c='green', s=100, label='Start', edgecolors='black')
    plt.scatter(goal[1], goal[0], c='red', s=100, label='Goal', edgecolors='black')
    
    plt.xticks(np.arange(0, grid.shape[1], 1))
    plt.yticks(np.arange(0, grid.shape[0], 1))
    plt.grid(True, which='both', color='black', linewidth=0.5)
    plt.legend()
    plt.title("A* Pathfinding with Obstacle Avoidance")
    plt.show()

# --- Main Execution ---

def main():
    resolution = 0.5  # meters per grid cell
    
    # Input
    start_input = input("Enter takeoff coordinates (lat,lon): ")
    goal_input = input("Enter landing coordinates (lat,lon): ")
    
    try:
        start_lat, start_lon = map(float, start_input.strip().split(","))
        goal_lat, goal_lon = map(float, goal_input.strip().split(","))
    except Exception:
        print("Invalid coordinates. Please use the format: lat,lon")
        return
    
    # Grid conversion
    start_grid = (0, 0)
    goal_grid = latlon_to_grid(start_lat, start_lon, goal_lat, goal_lon, resolution)
    
    print(f"Start Grid: {start_grid}, Goal Grid: {goal_grid}")
    
    # Define grid size
    grid_width = max(start_grid[0], goal_grid[0]) + 10
    grid_height = max(start_grid[1], goal_grid[1]) + 10
    grid = np.zeros((grid_height, grid_width))
    
    # Shift if necessary (in case of negative coordinates)
    shift_x = -min(0, start_grid[0], goal_grid[0])
    shift_y = -min(0, start_grid[1], goal_grid[1])
    start_grid = (start_grid[0] + shift_x, start_grid[1] + shift_y)
    goal_grid = (goal_grid[0] + shift_x, goal_grid[1] + shift_y)
    
    # Recalculate grid with margins
    grid_width = max(start_grid[0], goal_grid[0]) + 10
    grid_height = max(start_grid[1], goal_grid[1]) + 10
    grid = np.zeros((grid_height, grid_width))
    
    # Place obstacle exactly on the midline between start and goal
    mid_x = (start_grid[0] + goal_grid[0]) // 2
    mid_y = (start_grid[1] + goal_grid[1]) // 2
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            ox = mid_x + dx
            oy = mid_y + dy
            if 0 <= ox < grid.shape[1] and 0 <= oy < grid.shape[0]:
                grid[ox, oy] = 1  # Mark obstacle
    
    print(f"Obstacle placed around grid cell ({mid_x},{mid_y}).")
    
    # Find path
    path = find_path(grid, start_grid, goal_grid)
    
    if not path:
        print("No path found!")
        return
    
    # Convert path back to GPS
    gps_path = [grid_to_latlon(start_lat, start_lon, (p[0] - shift_x, p[1] - shift_y), resolution) for p in path]
    
    print("Waypoints (Latitude, Longitude):")
    for idx, (lat, lon) in enumerate(gps_path):
        print(f"{idx+1}: {lat:.7f}, {lon:.7f}")
    
    # Visualization
    visualize_path(grid, path, start_grid, goal_grid)

if __name__ == "__main__":
    main()
