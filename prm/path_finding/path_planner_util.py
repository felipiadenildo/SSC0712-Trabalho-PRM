# prm/path_finding/path_planner_util.py

import numpy as np
from nav_msgs.msg import OccupancyGrid

def convert_occupancy_grid_to_map(msg: OccupancyGrid):
    """ Converte uma mensagem OccupancyGrid em um mapa 2D numpy. """
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    # Inverte a matriz verticalmente porque o OccupancyGrid tem origem (0,0) no canto inferior esquerdo
    return np.flipud(data)

def make_walk_map(base_map, size):
    """
    Gera um mapa de "caminhabilidade" (walkable map) a partir de um mapa base.
    No mapa de retorno: 1 é caminhável, 0 não é.
    """
    h, w = base_map.shape
    # Considera células ocupadas (valor > 50) como obstáculos. O -1 (desconhecido) também será obstáculo.
    binary_obstacles = (base_map > 50).astype(np.uint8)
    
    padded = np.pad(binary_obstacles, size, constant_values=1)
    walkable = np.ones_like(base_map, dtype=np.uint8)

    for i in range(h):
        for j in range(w):
            if base_map[i, j] == -1: # Marca áreas desconhecidas como não-caminháveis
                walkable[i, j] = 0
                continue
            
            kernel = padded[i:i + 2 * size + 1, j:j + 2 * size + 1]
            if np.any(kernel == 1):
                walkable[i, j] = 0
    return walkable

def min_dist(walkable, ref, num_directions=32, max_radius=50):
    """ Calcula a distância mínima de um ponto a um obstáculo no mapa 'walkable'. """
    h, w = walkable.shape
    angles = np.linspace(0, 2 * np.pi, num_directions, endpoint=False)
    min_d = max_radius
    for theta in angles:
        for r in range(1, max_radius):
            i = int(round(ref[0] + r * np.sin(theta)))
            j = int(round(ref[1] + r * np.cos(theta)))
            if not (0 <= i < h and 0 <= j < w):
                break
            if walkable[i, j] == 0:
                min_d = min(min_d, r)
                break
    return min_d

def shell_points(ref, side_len, walkable, n=5):
    """ Encontra pontos em uma "casca" quadrada ao redor de um ponto de referência. """
    i, j = ref
    h, w = walkable.shape
    half = int(round(side_len))
    si, ei = max(0, i - half), min(h, i + half + 1)
    sj, ej = max(0, j - half), min(w, j + half + 1)

    def linspace_indices(start, end, count):
        if count <= 1: return [start] if count == 1 else []
        return [int(round(x)) for x in np.linspace(start, end-1, count)]

    top = [(si, col) for col in linspace_indices(sj, ej, n)]
    bottom = [(ei - 1, col) for col in linspace_indices(sj, ej, n)]
    left = [(row, sj) for row in linspace_indices(si, ei, n)]
    right = [(row, ej - 1) for row in linspace_indices(si, ei, n)]

    shell = set(top + bottom + left + right)
    return [pt for pt in shell if 0 <= pt[0] < h and 0 <= pt[1] < w and walkable[pt[0], pt[1]] == 1]

def find_path(base_map, start, goal, sz=5, num_points=5):
    """ Função principal que implementa o algoritmo de busca de caminho. """
    walkable_map = make_walk_map(base_map, size=sz)
    
    # Verifica se o início ou o fim estão em áreas não-caminháveis
    if not walkable_map[start]: return []
    if not walkable_map[goal]: return []
    
    max_steps = int(2 * np.linalg.norm(np.array(base_map.shape)))
    
    current = start
    back_reference = {start: None} # Dicionário para reconstruir o caminho
    shell_info = [] # Lista de tuplas (ponto, dist_goal, dist_obst)

    # Adiciona o ponto de partida à lista de processamento
    dist_goal = np.linalg.norm(np.array(start) - np.array(goal))
    shell_info.append((start, dist_goal))
    
    visited = {start}

    for step in range(max_steps):
        if not shell_info:
            print("Não há mais pontos para explorar.")
            return [] # Caminho não encontrado

        # Ordena pela distância ao objetivo
        shell_info.sort(key=lambda x: x[1])
        
        current, _ = shell_info.pop(0)

        # Verifica se chegou ao destino
        if np.linalg.norm(np.array(current) - np.array(goal)) < sz:
            back_reference[goal] = current
            break

        dist_obst = min_dist(walkable_map, current)
        side_len = int(np.ceil(dist_obst / np.sqrt(2)))
        
        raw_shell = shell_points(current, side_len, walkable_map, n=num_points)

        for pt in raw_shell:
            if pt in visited:
                continue
            
            visited.add(pt)
            dist_obst_pt = min_dist(walkable_map, pt)

            if dist_obst_pt < 3:
                continue
            
            dist_goal_pt = np.linalg.norm(np.array(pt) - np.array(goal))
            shell_info.append((pt, dist_goal_pt))
            back_reference[pt] = current
    else: # se o for terminar sem break
        print("Caminho não encontrado dentro do número máximo de passos.")
        return []

    # Reconstrução do caminho
    path = []
    curr = goal
    while curr is not None:
        path.append(curr)
        curr = back_reference.get(curr)
    
    return path[::-1] # Retorna o caminho do início para o fim