# prm/path_finding/path_planner_util.py

import numpy as np
from nav_msgs.msg import OccupancyGrid

def convert_occupancy_grid_to_map(msg: OccupancyGrid):
    """
    Converte uma mensagem OccupancyGrid em um mapa 2D numpy.
    Valores no mapa de retorno:
    - 0: Livre
    - 100: Ocupado
    - -1: Desconhecido
    """
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    # OccupancyGrid tem origem (0,0) no canto inferior esquerdo, numpy no superior esquerdo.
    # Inverter verticalmente corrige essa diferença.
    return np.flipud(data)

def make_walk_map(base_map, size):
    """
    Gera um mapa de "caminhabilidade" (walkable map) a partir de um mapa base.
    No mapa de retorno: 1 é caminhável, 0 não é.
    Esta função "engorda" os obstáculos para criar uma margem de segurança.

    Args:
        base_map: O mapa original do OccupancyGrid.
        size (int): O raio (em células) da margem de segurança.
    """
    h, w = base_map.shape
    # Considera células ocupadas (valor > 50) e desconhecidas (-1) como obstáculos.
    binary_obstacles = ((base_map > 50) | (base_map == -1)).astype(np.uint8)

    # Adiciona um preenchimento (padding) para evitar problemas nas bordas.
    padded = np.pad(binary_obstacles, size, constant_values=1)
    walkable = np.ones_like(base_map, dtype=np.uint8)

    # Itera sobre cada célula do mapa original.
    for i in range(h):
        for j in range(w):
            # Pega uma "janela" ao redor da célula atual.
            kernel = padded[i:i + 2 * size + 1, j:j + 2 * size + 1]
            # Se houver qualquer obstáculo nessa janela, marca a célula como não-caminhável.
            if np.any(kernel == 1):
                walkable[i, j] = 0
    return walkable

def min_dist(walkable, ref, num_directions=32, max_radius=50):
    """
    Calcula a distância mínima de um ponto de referência a um obstáculo,
    lançando "raios" em várias direções.
    """
    h, w = walkable.shape
    angles = np.linspace(0, 2 * np.pi, num_directions, endpoint=False)
    min_d = max_radius
    for theta in angles:
        for r in range(1, max_radius):
            i = int(round(ref[0] + r * np.sin(theta)))
            j = int(round(ref[1] + r * np.cos(theta)))
            if not (0 <= i < h and 0 <= j < w) or walkable[i, j] == 0:
                min_d = min(min_d, r)
                break
    return min_d

def shell_points(ref, side_len, walkable, n=5):
    """
    Encontra pontos caminháveis em uma "casca" quadrada ao redor de um ponto de referência.
    """
    i, j = ref
    h, w = walkable.shape
    half = int(round(side_len))
    si, ei = max(0, i - half), min(h, i + half + 1)
    sj, ej = max(0, j - half), min(w, j + half + 1)

    def linspace_indices(start, end, count):
        if count <= 1: return [start] if count == 1 else []
        return [int(round(x)) for x in np.linspace(start, end - 1, count)]

    top = [(si, col) for col in linspace_indices(sj, ej, n)]
    bottom = [(ei - 1, col) for col in linspace_indices(sj, ej, n)]
    left = [(row, sj) for row in linspace_indices(si, ei, n)]
    right = [(row, ej - 1) for row in linspace_indices(si, ei, n)]

    shell = set(top + bottom + left + right)
    return [pt for pt in shell if 0 <= pt[0] < h and 0 <= pt[1] < w and walkable[pt[0], pt[1]] == 1]

def simplify_path(path, walkable_matrix):
    """
    Reduz o número de pontos em um caminho, removendo waypoints intermediários
    se houver uma linha de visão direta e segura entre os pontos.
    """
    if len(path) < 3:
        return path[:]

    simplified = [path[0]]
    last_waypoint = path[0]

    for i in range(2, len(path)):
        current_waypoint = path[i - 1]
        next_waypoint = path[i]

        # Verifica se há uma linha reta "caminhável" entre o último waypoint adicionado e o próximo.
        last_x, last_y = last_waypoint
        next_x, next_y = next_waypoint

        # Extrai a submatriz entre os dois pontos para verificar obstáculos.
        x1, x2 = min(last_x, next_x), max(last_x, next_x)
        y1, y2 = min(last_y, next_y), max(last_y, next_y)

        # Adiciona uma pequena margem para a verificação.
        cutout = walkable_matrix[x1:x2 + 1, y1:y2 + 1]

        # Se há um obstáculo no caminho direto (célula == 0)...
        if np.any(cutout == 0):
            # ...então o waypoint anterior é necessário. Adiciona ele ao caminho.
            simplified.append(current_waypoint)
            last_waypoint = current_waypoint

    simplified.append(path[-1]) # Adiciona o ponto final.
    return simplified

def local_max_distance_point(pt, walkable, k=2):
    """
    Encontra o ponto mais seguro (mais distante de obstáculos) em uma vizinhança local.
    """
    h, w = walkable.shape
    i, j = pt
    x1, x2 = max(0, i - k), min(h, i + k + 1)
    y1, y2 = max(0, j - k), min(w, j + k + 1)
    
    sub_region = [(ii, jj) for ii in range(x1, x2) for jj in range(y1, y2) if walkable[ii, jj]]
    if not sub_region:
        return pt
        
    # Retorna o ponto da vizinhança que tem a maior distância mínima para um obstáculo.
    best_point = max(sub_region, key=lambda p: min_dist(walkable, p))
    return best_point

def adjust_path(path, walkable, k=2):
    """
    Aplica a otimização de segurança a todos os pontos do caminho.
    """
    return [local_max_distance_point(pt, walkable, k) for pt in path]

def find_path(base_map, start, goal, sz=5, num_points=5, adjust_size=2):
    """
    Função principal que implementa o algoritmo de busca de caminho.
    Combina a busca por um caminho inicial com etapas de otimização.
    """
    walkable_map = make_walk_map(base_map, size=sz)

    # Verifica se os pontos de início e fim são válidos.
    if not (0 <= start[0] < walkable_map.shape[0] and 0 <= start[1] < walkable_map.shape[1] and walkable_map[start]):
        print("Ponto de partida está em uma área não caminhável.")
        return []
    if not (0 <= goal[0] < walkable_map.shape[0] and 0 <= goal[1] < walkable_map.shape[1] and walkable_map[goal]):
        print("Ponto de chegada está em uma área não caminhável.")
        return []
    
    # Busca heurística por um caminho inicial (baseado em distância e segurança)
    max_steps = int(2 * np.linalg.norm(np.array(base_map.shape)))
    current = start
    back_reference = {start: None}
    
    visited = {start}
    # Fila de prioridade: (ponto, distância_ao_alvo)
    queue = [(start, np.linalg.norm(np.array(start) - np.array(goal)))]

    path_found = False
    for step in range(max_steps):
        if not queue:
            print("Caminho não encontrado: fila vazia.")
            return []

        queue.sort(key=lambda x: x[1]) # Ordena pela distância ao alvo
        current, _ = queue.pop(0)

        if np.linalg.norm(np.array(current) - np.array(goal)) < sz:
            back_reference[goal] = current
            path_found = True
            break

        dist_obst = min_dist(walkable_map, current)
        side_len = int(np.ceil(dist_obst / np.sqrt(2)))
        
        shell = shell_points(current, side_len, walkable_map, n=num_points)

        for pt in shell:
            if pt not in visited:
                visited.add(pt)
                dist_goal = np.linalg.norm(np.array(pt) - np.array(goal))
                queue.append((pt, dist_goal))
                back_reference[pt] = pt # Correção: deveria ser 'current'
                back_reference[pt] = current


    if not path_found:
        print("Caminho não encontrado dentro do número máximo de passos.")
        return []

    # Reconstrução do caminho bruto
    path = []
    curr = goal
    while curr is not None:
        path.append(curr)
        curr = back_reference.get(curr)
    path = path[::-1]

    # Etapas de otimização do caminho
    path_adjusted = adjust_path(path, walkable_map, k=adjust_size)
    path_simplified = simplify_path(path_adjusted, walkable_map)
    final_path = adjust_path(path_simplified, walkable_map, k=adjust_size)

    return final_path