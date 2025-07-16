import numpy as np

def make_walk_map(base_map, size):
    h, w = base_map.shape
    padded = np.pad(base_map, size, constant_values=0)
    walkable = np.ones_like(base_map, dtype=np.uint8)
    for i in range(h):
        for j in range(w):
            kernel = padded[i:i + 2 * size + 1, j:j + 2 * size + 1]
            if np.any(kernel == 1.0):
                walkable[i, j] = 0
    return walkable

def min_dist(walkable, ref, num_directions=64, max_radius=50):
    h, w = walkable.shape
    angles = np.linspace(0, 2 * np.pi, num_directions, endpoint=False)
    min_dist = max_radius
    for theta in angles:
        for r in range(1, max_radius):
            i = int(round(ref[0] + r * np.sin(theta)))
            j = int(round(ref[1] + r * np.cos(theta)))
            if not (0 <= i < h and 0 <= j < w):
                break
            if walkable[i, j] == 0:
                min_dist = min(min_dist, r)
                break
    return min_dist

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

    top_cols = linspace_indices(sj, ej - 1, n)
    top = [(si, col) for col in top_cols]

    bottom_cols = linspace_indices(sj, ej - 1, n)
    bottom = [(ei - 1, col) for col in bottom_cols]

    left_rows = linspace_indices(si, ei - 1, n)
    left = [(row, sj) for row in left_rows]

    right_rows = linspace_indices(si, ei - 1, n)
    right = [(row, ej - 1) for row in right_rows]


    shell = top + bottom + left + right
    return shell



# ===== EXECUÇÃO =====
def find_path(base_map, sz =5, start = (50,50), goal = (50,50), num_points = 5):
    

    walkable_map = make_walk_map(base_map, size = sz)


    current = start
    back = 0          #reference
    shell_info = []   #track points not checked
    track = []        #tracks points checked


    for step in range(100):

        track.append((current, back))

        dist_obst = min_dist(walkable_map, current)
        side_len = int(np.ceil((dist_obst) / np.sqrt(2)))

        raw_shell = shell_points(current, side_len, walkable_map, n=num_points)
        if not raw_shell:
            break

        for pt in raw_shell:
            dist_obst = min_dist(walkable_map, pt)

            if dist_obst < 3:
                #Adiciona apenas pontos com distancia suficiente
                continue
            
            dist_goal = np.linalg.norm(np.array(pt) - np.array(goal))
            shell_info.append((pt, dist_goal, dist_obst, step))

        # Ordena somente pela distância ao objetivo (dist_goal) — que é índice 1 no tuple
        shell_info.sort(key=lambda x: x[1])

        if not shell_info:
            break

        current = shell_info[0][0]
        back = shell_info[0][3]
        
        shell_info.pop(0)

        if shell_info[0][1] < shell_info[0][2]:
            track.append((current,back))
            track.append((goal,step+1))
            break


    path = []
    s = step+2

    while(s != track[s][1]):
        print(s," ",track[s][1])
        path.append(track[s][0])
        s = track[s][1]
      
    path.append(start)
    

    return path