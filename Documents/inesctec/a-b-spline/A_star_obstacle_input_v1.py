import numpy as np
import matplotlib.pyplot as plt
import heapq
from typing import List, Tuple, Dict
from math import sqrt, cos, radians

# ---------- A* Implementation ----------
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(array, start, goal):
    neighbors = [(0,1),(1,0),(0,-1),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0]+i, current[1]+j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
                
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

# ---------- Helper Functions ----------
def latlon_to_grid(lat0, lon0, lat, lon, resolution):
    # Approximate conversion using 111111 m per degree latitude and longitude scaled by cos(latitude)
    dlat = (lat - lat0) * 111111
    dlon = (lon - lon0) * 111111 * np.cos(np.radians(lat0))
    x = int(dlat / resolution)
    y = int(dlon / resolution)
    return x, y

def place_obstacle(grid, obs_grid, size=2):
    ox, oy = obs_grid
    for dx in range(-size//2, size//2 + 1):
        for dy in range(-size//2, size//2 + 1):
            x, y = ox + dx, oy + dy
            if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                grid[x, y] = 1

def grid_to_latlon(lat0, lon0, gx, gy, resolution):
    dlat = gx * resolution / 111111
    dlon = gy * resolution / (111111 * np.cos(np.radians(lat0)))
    lat = lat0 + dlat
    lon = lon0 + dlon
    return lat, lon


def generate_waypoints_file(gps_path: List[Tuple[float, float]], altitude: float = 10.0, filename: str = "mission_obstacles.waypoints"):
    with open(filename, "w") as file:
        file.write("QGC WPL 110\n")
        for idx, (lat, lon) in enumerate(gps_path):
            file.write(f"{idx}\t0\t3\t16\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude:.2f}\t1\n")
    print(f"Waypoints saved to '{filename}'.")

# ---------- Main ----------
def main():
    # Resolution and grid size
    resolution = 1  # metres per grid cell
    grid_size = 100   # grid dimensions (100x100 cells)

    # Ask user for start and goal coordinates
    start_input = input("Enter start coordinates (lat,lon): ")
    goal_input = input("Enter goal coordinates (lat,lon): ")
    start_lat, start_lon = map(float, start_input.split(","))
    goal_lat, goal_lon = map(float, goal_input.split(","))

    # Base reference point (start as origin)
    ref_lat, ref_lon = start_lat, start_lon

    # Convert to grid coordinates
    start = latlon_to_grid(ref_lat, ref_lon, start_lat, start_lon, resolution)
    goal = latlon_to_grid(ref_lat, ref_lon, goal_lat, goal_lon, resolution)

    # Shift so coordinates are positive in the grid
    shift_x = grid_size // 2
    shift_y = grid_size // 2
    start = (start[0] + shift_x, start[1] + shift_y)
    goal = (goal[0] + shift_x, goal[1] + shift_y)

    # Initialise grid
    grid = np.zeros((grid_size, grid_size))

    # Ask for obstacles
    num_obs = int(input("Enter number of obstacles: "))
    obstacles = []
    for i in range(num_obs):
        obs_input = input(f"Enter obstacle {i+1} coordinates (lat,lon): ")
        obs_lat, obs_lon = map(float, obs_input.split(","))
        obs_grid = latlon_to_grid(ref_lat, ref_lon, obs_lat, obs_lon, resolution)
        obs_grid = (obs_grid[0] + shift_x, obs_grid[1] + shift_y)
        obstacles.append(obs_grid)
        place_obstacle(grid, obs_grid, size=4)  # obstacle size (in grid cells)

    # Run A*
    path = astar(grid, start, goal)

    # ---------- Visualisation ----------
        # Plot
    plt.imshow(grid.T, origin='lower', cmap='gray_r')

    if path:
        px, py = zip(*path)
        plt.plot(px, py, 'b-', linewidth=2, label="Path")

        # Adjust view dynamically to bounding box of path, start, goal, obstacles
        all_x = list(px) + [start[0], goal[0]] + [ox for ox, _ in obstacles]
        all_y = list(py) + [start[1], goal[1]] + [oy for _, oy in obstacles]

        margin = 5  # cells of padding around trajectory
        plt.xlim(min(all_x) - margin, max(all_x) + margin)
        plt.ylim(min(all_y) - margin, max(all_y) + margin)

        gps_path = [grid_to_latlon(ref_lat, ref_lon, x - shift_x, y - shift_y, resolution) for x, y in path]
        generate_waypoints_file(gps_path, altitude=10.0, filename="mission_obstacles.waypoints")

    plt.plot(start[0], start[1], "go", label="Start")
    plt.plot(goal[0], goal[1], "ro", label="Goal")
    for ox, oy in obstacles:
        plt.plot(ox, oy, "ks", label="Obstacle" if (ox, oy) == obstacles[0] else "")

    plt.legend()
    plt.title("A* Pathfinding with Obstacles (Zoomed)")
    plt.show()


if __name__ == "__main__":
    main()
