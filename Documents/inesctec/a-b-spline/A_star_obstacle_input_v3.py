# ============================================================
#  Theta* Path Planning with Cubic-Spline Smoothing
#  Author: Mr. Barros + GPT-5
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy.interpolate import CubicSpline
import random

# ---------- Helper Functions ----------
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

def line_of_sight(grid, p1, p2):
    for (x, y) in bresenham_line(p1[0], p1[1], p2[0], p2[1]):
        if grid[x, y] == 1:
            return False
    return True

# ---------- Theta* Implementation ----------
def theta_star(grid, start, goal):
    neighbors = [(-1,-1), (-1,0), (-1,1),
                 (0,-1),          (0,1),
                 (1,-1),  (1,0),  (1,1)]
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), start))
    came_from = {start: start}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while True:
                path.append(current)
                if current == start:
                    break
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]):
                continue
            if grid[neighbor[0], neighbor[1]] == 1:
                continue

            parent = came_from[current]
            if line_of_sight(grid, parent, neighbor):
                tentative_g = g_score[parent] + heuristic(parent, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = parent
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
            else:
                tentative_g = g_score[current] + heuristic(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

# ---------- Coordinate Conversion ----------
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

# ---------- Grid and Waypoint Utilities ----------
def place_obstacle(grid, obs_grid, size=2):
    ox, oy = obs_grid
    for dx in range(-size//2, size//2 + 1):
        for dy in range(-size//2, size//2 + 1):
            x, y = ox + dx, oy + dy
            if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                grid[x, y] = 1

def generate_waypoints_file(gps_points, altitude=10.0, filename="mission_theta.waypoints"):
    with open(filename, "w") as f:
        f.write("QGC WPL 110\n")
        f.write(f"0\t1\t0\t22\t0\t0\t0\t0\t{gps_points[0][0]:.7f}\t{gps_points[0][1]:.7f}\t{altitude}\t1\n")
        f.write(f"1\t0\t3\t22\t0\t0\t0\t0\t{gps_points[0][0]:.7f}\t{gps_points[0][1]:.7f}\t{altitude}\t1\n")
        for i, (lat, lon) in enumerate(gps_points):
            f.write(f"{i+2}\t0\t3\t16\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude}\t1\n")
        f.write(f"{len(gps_points)+2}\t0\t3\t20\t0\t0\t0\t0\t{gps_points[-1][0]:.7f}\t{gps_points[-1][1]:.7f}\t{altitude}\t1\n")
    print(f"\nWaypoints saved to: {filename}")

# ---------- Main ----------
def main():
    resolution = 1.0
    grid_size = 100

    # User input
    start_input = input("Enter start coordinates (lat,lon): ")
    goal_input = input("Enter goal coordinates (lat,lon): ")
    start_lat, start_lon = map(float, start_input.split(","))
    goal_lat, goal_lon = map(float, goal_input.split(","))

    n_obs = int(input("Enter number of obstacles: "))

    ref_lat, ref_lon = start_lat, start_lon
    start_grid = latlon_to_grid(ref_lat, ref_lon, start_lat, start_lon, resolution)
    goal_grid = latlon_to_grid(ref_lat, ref_lon, goal_lat, goal_lon, resolution)

    shift_x = grid_size // 2
    shift_y = grid_size // 2
    start_grid = (start_grid[0] + shift_x, start_grid[1] + shift_y)
    goal_grid = (goal_grid[0] + shift_x, goal_grid[1] + shift_y)

    grid = np.zeros((grid_size, grid_size), dtype=int)
    obstacles = []

    # Random obstacle placement near line connecting start and goal
    for _ in range(n_obs):
        t = random.uniform(0.3, 0.7)  # somewhere along the middle segment
        x = int(start_grid[0] + t * (goal_grid[0] - start_grid[0]))
        y = int(start_grid[1] + t * (goal_grid[1] - start_grid[1]))
        # add random offset
        x += random.randint(-5, 5)
        y += random.randint(-5, 5)
        obstacles.append((x, y))
        place_obstacle(grid, (x, y), size=4)

    # Theta* Path
    path = theta_star(grid, start_grid, goal_grid)
    if not path:
        print("No path found.")
        return

    # Plot
    plt.imshow(grid.T, origin='lower', cmap='gray_r')
    px, py = zip(*path)
    plt.plot(px, py, 'b--', linewidth=1.5, label="Theta* Path")

    # Cubic Spline smoothing
    t = np.linspace(0, 1, len(px))
    csx = CubicSpline(t, px)
    csy = CubicSpline(t, py)
    t_smooth = np.linspace(0, 1, 200)
    smooth_x = csx(t_smooth)
    smooth_y = csy(t_smooth)
    plt.plot(smooth_x, smooth_y, 'darkorange', linewidth=2.5, label="Smoothed Path")

    # Dynamic zoom
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
    plt.title("Theta* with Random Obstacles & Cubic-Spline Smoothing")
    plt.show()

    # Convert to GPS and export
    gps_path = [grid_to_latlon(ref_lat, ref_lon, x - shift_x, y - shift_y, resolution)
                for x, y in zip(smooth_x, smooth_y)]
    generate_waypoints_file(gps_path, altitude=10.0, filename="mission_theta.waypoints")

# ------------------------------------------------------------
if __name__ == "__main__":
    main()
