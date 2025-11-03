# ============================================================
#  A* Path Planning with Cubic-Spline Smoothing
#  Author: Mr. Barros + GPT-5
#  Description:
#     - Requests start & goal coordinates (lat, lon)
#     - Allows user to define any number of obstacles
#     - Computes an A* path on a 2-D grid
#     - Smooths it using a cubic spline
#     - Exports a Mission Planner–compatible .waypoints file
#     - Displays both raw and smoothed trajectories
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy.interpolate import CubicSpline

# ------------------------------------------------------------
# 1.  A* Search
# ------------------------------------------------------------
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
            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                if array[neighbor[0]][neighbor[1]] == 1:
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

# ------------------------------------------------------------
# 2.  Coordinate Conversion
# ------------------------------------------------------------
def latlon_to_grid(lat0, lon0, lat, lon, resolution):
    dlat = (lat - lat0) * 111111
    dlon = (lon - lon0) * 111111 * np.cos(np.radians(lat0))
    x = int(dlat / resolution)
    y = int(dlon / resolution)
    return x, y

def grid_to_latlon(lat0, lon0, gx, gy, resolution):
    dlat = gx * resolution / 111111
    dlon = gy * resolution / (111111 * np.cos(np.radians(lat0)))
    lat = lat0 + dlat
    lon = lon0 + dlon
    return lat, lon

# ------------------------------------------------------------
# 3.  Grid Utilities
# ------------------------------------------------------------
def place_obstacle(grid, obs_grid, size=2):
    ox, oy = obs_grid
    for dx in range(-size//2, size//2 + 1):
        for dy in range(-size//2, size//2 + 1):
            x, y = ox + dx, oy + dy
            if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                grid[x, y] = 1

# ------------------------------------------------------------
# 4.  Waypoint Export (Mission Planner format)
# ------------------------------------------------------------
def generate_waypoints_file(gps_points, altitude=10.0, filename="mission_spline.waypoints"):
    with open(filename, "w") as f:
        f.write("QGC WPL 110\n")
        f.write(f"0\t1\t0\t22\t0\t0\t0\t0\t{gps_points[0][0]:.7f}\t{gps_points[0][1]:.7f}\t{altitude}\t1\n")  # Home
        f.write(f"1\t0\t3\t22\t0\t0\t0\t0\t{gps_points[0][0]:.7f}\t{gps_points[0][1]:.7f}\t{altitude}\t1\n")  # Takeoff
        for i, (lat, lon) in enumerate(gps_points):
            f.write(f"{i+2}\t0\t3\t16\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude}\t1\n")
        f.write(f"{len(gps_points)+2}\t0\t3\t20\t0\t0\t0\t0\t{gps_points[-1][0]:.7f}\t{gps_points[-1][1]:.7f}\t{altitude}\t1\n")  # RTL
    print(f"\nWaypoints saved to: {filename}")

# ------------------------------------------------------------
# 5.  Main
# ------------------------------------------------------------
def main():
    resolution = 1.0   # metres per grid cell
    grid_size = 100

    # --- Input section ---
    start_input = input("Enter start coordinates (lat,lon): ")
    goal_input = input("Enter goal coordinates (lat,lon): ")
    start_lat, start_lon = map(float, start_input.split(","))
    goal_lat, goal_lon = map(float, goal_input.split(","))

    n_obs = int(input("Enter number of obstacles: "))
    obstacles_coords = []
    for i in range(n_obs):
        obs_input = input(f"Enter obstacle {i+1} coordinates (lat,lon): ")
        obs_lat, obs_lon = map(float, obs_input.split(","))
        obstacles_coords.append((obs_lat, obs_lon))

    ref_lat, ref_lon = start_lat, start_lon
    start_grid = latlon_to_grid(ref_lat, ref_lon, start_lat, start_lon, resolution)
    goal_grid = latlon_to_grid(ref_lat, ref_lon, goal_lat, goal_lon, resolution)

    shift_x = grid_size // 2
    shift_y = grid_size // 2
    start_grid = (start_grid[0] + shift_x, start_grid[1] + shift_y)
    goal_grid = (goal_grid[0] + shift_x, goal_grid[1] + shift_y)

    grid = np.zeros((grid_size, grid_size))
    obstacles = []
    for obs_lat, obs_lon in obstacles_coords:
        obs_grid = latlon_to_grid(ref_lat, ref_lon, obs_lat, obs_lon, resolution)
        obs_grid = (obs_grid[0] + shift_x, obs_grid[1] + shift_y)
        obstacles.append(obs_grid)
        place_obstacle(grid, obs_grid, size=4)

    # --- A* computation ---
    path = astar(grid, start_grid, goal_grid)
    if not path:
        print("No path found.")
        return

    # --- Plot and smoothing ---
    plt.imshow(grid.T, origin='lower', cmap='gray_r')

    px, py = zip(*path)
    plt.plot(px, py, 'b--', linewidth=1.5, label="A* Path")

    # Spline smoothing
    t = np.linspace(0, 1, len(px))
    cs_x = CubicSpline(t, px)
    cs_y = CubicSpline(t, py)
    t_smooth = np.linspace(0, 1, 200)
    smooth_x = cs_x(t_smooth)
    smooth_y = cs_y(t_smooth)

    plt.plot(smooth_x, smooth_y, 'darkorange', linewidth=2.5, label="Smoothed Path")

    all_x = list(px) + [start_grid[0], goal_grid[0]] + [ox for ox, _ in obstacles]
    all_y = list(py) + [start_grid[1], goal_grid[1]] + [oy for _, oy in obstacles]
    margin = 5
    plt.xlim(min(all_x) - margin, max(all_x) + margin)
    plt.ylim(min(all_y) - margin, max(all_y) + margin)

    plt.plot(start_grid[0], start_grid[1], "go", label="Start")
    plt.plot(goal_grid[0], goal_grid[1], "ro", label="Goal")
    for ox, oy in obstacles:
        plt.plot(ox, oy, "ks", label="Obstacle" if (ox, oy)==obstacles[0] else "")

    plt.legend()
    plt.title("A* with Cubic-Spline Smoothing (Zoomed)")
    plt.show()

    # --- Convert to GPS + Export ---
    gps_path = [grid_to_latlon(ref_lat, ref_lon, x - shift_x, y - shift_y, resolution)
                for x, y in zip(smooth_x, smooth_y)]
    generate_waypoints_file(gps_path, altitude=10.0, filename="mission_spline.waypoints")

# ------------------------------------------------------------
if __name__ == "__main__":
    main()
