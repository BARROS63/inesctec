import math
import heapq

# --- CONFIGURAÇÕES GERAIS ---
# Tamanho do mapa (metros)
MAP_LENGTH = 8  # comprimento (eixo x)
MAP_WIDTH = 4   # largura (eixo y)
GRID_RESOLUTION = 1.0  # metros por célula

# Coordenadas iniciais reais
LAT_START = 41.179516
LON_START = -8.595673

# Ponto de destino relativo ao tamanho do retângulo
goal = (int(MAP_LENGTH / GRID_RESOLUTION) - 1, int(MAP_WIDTH / GRID_RESOLUTION) - 1)

# Funções auxiliares para converter deslocamento em metros para coordenadas GPS
def meters_to_lat(meters):
    return meters / 111111  # Aproximação válida para pequenas distâncias

def meters_to_lon(meters, latitude):
    return meters / (111111 * math.cos(math.radians(latitude)))

# --- AMBIENTE COM OBSTÁCULO ---
def create_obstacle():
    obstacles = set()
    # Criamos um bloco de obstáculos no centro do mapa
    obstacle_x = int(MAP_LENGTH / (2 * GRID_RESOLUTION))
    obstacle_y = int(MAP_WIDTH / (2 * GRID_RESOLUTION))
    for dx in range(-1, 2):  # Obstáculo de 3x3 células
        for dy in range(-1, 2):
            obstacles.add((obstacle_x + dx, obstacle_y + dy))
    return obstacles

# --- ALGORITMO A* ---
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(node, max_x, max_y):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dx, dy in directions:
        nx, ny = node[0] + dx, node[1] + dy
        if 0 <= nx < max_x and 0 <= ny < max_y:
            neighbors.append((nx, ny))
    return neighbors

def a_star(start, goal, obstacles, max_x, max_y):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstrução do caminho
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in get_neighbors(current, max_x, max_y):
            if neighbor in obstacles:
                continue

            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None  # Se não encontrar caminho

# --- CONVERSÃO PARA COORDENADAS GPS ---
def path_to_gps(path):
    gps_path = []
    for (x, y) in path:
        dx = x * GRID_RESOLUTION
        dy = y * GRID_RESOLUTION
        lat = LAT_START + meters_to_lat(dy)
        lon = LON_START + meters_to_lon(dx, LAT_START)
        gps_path.append((lat, lon))
    return gps_path

# --- EXECUÇÃO ---
def main():
    print("Planeamento de rota autónomo iniciado.")

    start = (0, 0)
    obstacles = create_obstacle()

    max_x = int(MAP_LENGTH / GRID_RESOLUTION)
    max_y = int(MAP_WIDTH / GRID_RESOLUTION)

    path = a_star(start, goal, obstacles, max_x, max_y)

    if path is None:
        print("Nenhum caminho encontrado!")
        return

    gps_path = path_to_gps(path)

    print("Caminho encontrado (coordenadas GPS):")
    for idx, (lat, lon) in enumerate(gps_path):
        print(f"Waypoint {idx + 1}: Latitude {lat:.7f}, Longitude {lon:.7f}")

    # Opcional: exportar para ficheiro para carregar no MAVProxy como missão
    with open("waypoints.txt", "w") as f:
        for lat, lon in gps_path:
            f.write(f"{lat},{lon}\n")

    print("Waypoints exportados para 'waypoints.txt'.")

if __name__ == "__main__":
    main()
