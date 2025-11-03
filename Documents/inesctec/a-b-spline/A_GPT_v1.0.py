from typing import List, Tuple, Dict, Set
import numpy as np
import heapq
from math import sqrt, cos, radians
import matplotlib.pyplot as plt

# Funções de A* já existentes:
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
        (x+1, y), (x-1, y),    # Right, Left
        (x, y+1), (x, y-1),    # Up, Down
        (x+1, y+1), (x-1, y-1),# Diagonals
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
    return []  # Caminho não encontrado

def visualize_path(grid: np.ndarray, path: List[Tuple[int, int]]):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='binary')
    if path:
        path_np = np.array(path)
        plt.plot(path_np[:, 1], path_np[:, 0], 'b-', linewidth=3, label='Path')
        plt.plot(path_np[0, 1], path_np[0, 0], 'go', markersize=10, label='Start')
        plt.plot(path_np[-1, 1], path_np[-1, 0], 'ro', markersize=10, label='Goal')
    plt.legend()
    plt.title("A* Pathfinding Result")
    plt.show()

# Funções para conversão entre coordenadas GPS e grid:
# Para pequenas distâncias, usamos aproximações:
# 1 grau de latitude ~ 111320 metros
# 1 grau de longitude ~ 111320 * cos(latitude) metros

def latlon_to_grid(start_lat: float, start_lon: float, lat: float, lon: float, resolution: float) -> Tuple[int, int]:
    # Diferença em metros (usando a latitude do ponto de partida como referência)
    dy = (lat - start_lat) * 111320
    dx = (lon - start_lon) * (111320 * cos(radians(start_lat)))
    # Converte para células do grid
    grid_x = int(round(dx / resolution))
    grid_y = int(round(dy / resolution))
    return grid_x, grid_y

def grid_to_latlon(start_lat: float, start_lon: float, grid_point: Tuple[int, int], resolution: float) -> Tuple[float, float]:
    dx = grid_point[0] * resolution
    dy = grid_point[1] * resolution
    lat = start_lat + (dy / 111320)
    lon = start_lon + (dx / (111320 * cos(radians(start_lat))))
    return lat, lon

def main():
    resolution = 1.0  # 1 metro por célula
    
    # Entrada de coordenadas reais
    start_input = input("Insira as coordenadas de decolagem (lat,lon): ")
    goal_input = input("Insira as coordenadas de pouso (lat,lon): ")
    
    try:
        start_lat, start_lon = map(float, start_input.split(","))
        goal_lat, goal_lon = map(float, goal_input.split(","))
    except Exception as e:
        print("Erro ao ler as coordenadas. Certifique-se de usar o formato: lat,lon")
        return
    
    # Converter as coordenadas para posições no grid
    start_grid = (0, 0)  # Usamos o ponto de decolagem como origem (0,0)
    goal_grid = latlon_to_grid(start_lat, start_lon, goal_lat, goal_lon, resolution)
    print(f"Start (grid): {start_grid}, Goal (grid): {goal_grid}")
    
    # Definir as dimensões do grid de modo que cubra do start ao goal com margem
    grid_width = abs(goal_grid[0]) + 10
    grid_height = abs(goal_grid[1]) + 10
    grid = np.zeros((grid_height, grid_width))
    
    # Posiciona o start no grid (mantemos em (0,0))
    # Ajusta o goal se estiver em posições negativas:
    shift_x = -min(0, start_grid[0], goal_grid[0])
    shift_y = -min(0, start_grid[1], goal_grid[1])
    start_grid = (start_grid[0] + shift_x, start_grid[1] + shift_y)
    goal_grid = (goal_grid[0] + shift_x, goal_grid[1] + shift_y)
    
    # Recria o grid com as novas dimensões, se necessário
    grid = np.zeros((grid_height, grid_width))
    
    # Coloca um obstáculo no meio da linha reta entre start e goal (3x3 células)
    mid_x = (start_grid[0] + goal_grid[0]) // 2
    mid_y = (start_grid[1] + goal_grid[1]) // 2
    for i in range(mid_y - 1, mid_y + 2):
        for j in range(mid_x - 1, mid_x + 2):
            if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1]:
                grid[i, j] = 1  # marca como obstáculo
    
    print(f"Obstáculo posicionado em (grid): aproximadamente ({mid_x}, {mid_y})")
    
    # Encontrar o caminho usando A*
    path_grid = find_path(grid, start_grid, goal_grid)
    if not path_grid:
        print("Nenhum caminho foi encontrado!")
        return
    
    # Converter o caminho do grid para coordenadas GPS reais
    path_gps = [grid_to_latlon(start_lat, start_lon, point, resolution) for point in path_grid]
    
    print("Waypoints (Latitude, Longitude):")
    for idx, (lat, lon) in enumerate(path_gps):
        print(f"{idx+1}: ({lat:.7f}, {lon:.7f})")
    
    # Visualiza o resultado
    visualize_path(grid, path_grid)

if __name__ == "__main__":
    main()
