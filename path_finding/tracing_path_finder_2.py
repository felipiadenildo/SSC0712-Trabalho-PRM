import numpy as np
from nav_msgs.msg import OccupancyGrid

def convert_occupancy_grid(msg: OccupancyGrid):
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    return data

def make_walk_map(base_map, size):
    """
    Gera mapa de caminhada com segurança baseada na distância a obstáculos.

    base_map: matriz 2D com:
        -1: desconhecido
         0: livre
        >0: ocupado (ex: 100)
    
    Retorna matriz com:
        1 → caminhável
        0 → não caminhável (próximo de obstáculo ou desconhecido)
    """
    h, w = base_map.shape

    # Máscara binária com 1 onde há obstáculo
    binary_obstacles = (base_map > 0).astype(np.uint8)

    # Padding para bordas
    padded = np.pad(binary_obstacles, size, constant_values=0)

    # Inicializa como tudo caminhável
    walkable = np.ones((h, w), dtype=np.uint8)

    for i in range(h):
        for j in range(w):
            kernel = padded[i:i + 2 * size + 1, j:j + 2 * size + 1]
            if np.any(kernel == 1):
                walkable[i, j] = 0  # Muito próximo de obstáculo
            elif base_map[i, j] == -1:
                walkable[i, j] = 0  # Desconhecido
            else:
                walkable[i, j] = 1  # Livre e seguro

    return walkable


def min_dist(walkable, ref, num_directions=32, max_radius=50):
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
    i, j = ref
    h, w = walkable.shape
    half = int(round(side_len))
    si, ei = max(0, i - half), min(h, i + half + 1)
    sj, ej = max(0, j - half), min(w, j + half + 1)

    def linspace_indices(start, end, count):
        if count == 1:
            return [start]
        return [int(round(x)) for x in np.linspace(start, end, count)]

    top = [(si, col) for col in linspace_indices(sj, ej - 1, n)]
    bottom = [(ei - 1, col) for col in linspace_indices(sj, ej - 1, n)]
    left = [(row, sj) for row in linspace_indices(si, ei - 1, n)]
    right = [(row, ej - 1) for row in linspace_indices(si, ei - 1, n)]

    shell = top + bottom + left + right
    return list({pt for pt in shell if 0 <= pt[0] < h and 0 <= pt[1] < w and walkable[pt] == 1})

def simplify_path(path, matrix):
    if len(path) < 3:
        return path[:]

    simplified = [path[0]]
    last = path[0]

    for i in range(2, len(path)):
        previous = path[i - 1]
        check = path[i]

        lx, ly = last
        cx, cy = check

        x1 = min(lx, cx)
        x2 = max(lx, cx)
        y1 = min(ly, cy)
        y2 = max(ly, cy)

        cutout = matrix[x1 - 1:x2 + 2, y1 - 1:y2 + 2]
        if np.any(cutout == 0):
            simplified.append(previous)
            last = previous

    simplified.append(path[-1])
    return simplified

def local_max_distance_point(pt, walkable, k=2):
    h, w = walkable.shape
    i, j = pt
    x1, x2 = max(0, i - k), min(h, i + k + 1)
    y1, y2 = max(0, j - k), min(w, j + k + 1)
    sub = [(ii, jj) for ii in range(x1, x2) for jj in range(y1, y2) if walkable[ii, jj]]
    if not sub:
        return pt
    best = max(sub, key=lambda p: min_dist(walkable, p))
    return best

def adjust_path(path, walkable, k=2):
    return [local_max_distance_point(pt, walkable, k) for pt in path]

def find_path(base_map, start, goal, sz=5, num_points=5, adjust_size=2):
    max_steps = int(2 * np.linalg.norm(base_map.shape))
    walkable_map = make_walk_map(base_map, size=sz)

    current = start
    back = 0
    shell_info = []
    track = []

    for step in range(max_steps):
        track.append((current, back))
        dist_obst = min_dist(walkable_map, current)
        side_len = int(np.ceil(dist_obst / np.sqrt(2)))
        raw_shell = shell_points(current, side_len, walkable_map, n=num_points)
        if not raw_shell:
            break

        for pt in raw_shell:
            dist_obst = min_dist(walkable_map, pt)
            if dist_obst < 3:
                continue
            dist_goal = np.linalg.norm(np.array(pt) - np.array(goal))
            shell_info.append((pt, dist_goal, dist_obst, step))

        shell_info.sort(key=lambda x: x[1])
        if not shell_info:
            break

        current = shell_info[0][0]
        back = shell_info[0][3]
        shell_info.pop(0)

        if shell_info and shell_info[0][1] < shell_info[0][2]:
            track.append((current, back))
            track.append((goal, step + 1))
            break

    # Reconstroi o caminho base
    path = []
    s = step + 2
    while s != track[s][1]:
        path.append(track[s][0])
        s = track[s][1]
    path.append(start)
    path = path[::-1]

    # Gera versões ajustadas e simplificadas
    path_2 = adjust_path(path, walkable_map, k=adjust_size)
    path_3 = simplify_path(path_2, walkable_map)
    path_4 = adjust_path(path_3, walkable_map, k=adjust_size)

    return path_4


