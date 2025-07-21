import heapq
import numpy as np
import cv2 # Usaremos a biblioteca OpenCV para uma inflação eficiente

class AStar:
    """
    Uma classe para encapsular o algoritmo de busca de caminho A*,
    com suporte para inflação de obstáculos para segurança.
    """
    def __init__(self, map_data, safety_radius_cells=3):
        """
        Construtor do A*.

        Args:
            map_data (np.array): Matriz 2D numpy representando o mapa.
            safety_radius_cells (int): O número de células (píxeis) para inflar
                                       à volta de cada obstáculo.
        """
        self.original_map = map_data
        self.height, self.width = map_data.shape
        self.safety_radius = safety_radius_cells

        # O planeamento agora será feito num mapa "inflacionado"
        self.inflated_map = self._inflate_obstacles()

    def _inflate_obstacles(self):
        """
        Cria uma margem de segurança à volta dos obstáculos no mapa.
        """
        # Encontra todos os pontos que são obstáculos (valor > 50)
        obstacle_mask = (self.original_map > 50).astype(np.uint8)

        # Usa a função 'dilate' do OpenCV, que é extremamente eficiente para esta tarefa.
        # Ela "engrossa" as áreas brancas (os nossos obstáculos) na máscara.
        # O 'kernel' define o tamanho da inflação. Um kernel de (2*r+1) x (2*r+1)
        # cria uma margem de 'r' células.
        kernel_size = 2 * self.safety_radius + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        inflated_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

        # Cria uma cópia do mapa original e aplica a máscara inflacionada.
        inflated_map = self.original_map.copy()
        inflated_map[inflated_mask == 1] = 100 # Marca as áreas inflacionadas como obstáculos
        
        print(f"[A*] Mapa inflacionado com um raio de segurança de {self.safety_radius} células.")
        return inflated_map

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def get_neighbors(self, node):
        """
        Obtém os vizinhos usando o MAPA INFLACIONADO.
        """
        neighbors = []
        for dx, dy, cost in [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
                             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]:
            
            x, y = node[0] + dx, node[1] + dy

            if 0 <= x < self.width and 0 <= y < self.height:
                # A verificação agora é feita no mapa com a margem de segurança
                if self.inflated_map[y, x] <= 50:
                    neighbors.append(((x, y), cost))
        return neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def find_path(self, start, goal):
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g_score = current_g + move_cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
        
        return None