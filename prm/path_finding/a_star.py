# prm/path_finding/a_star.py

import heapq
import numpy as np

class AStar:
    """
    Uma classe para encapsular o algoritmo de busca de caminho A*.
    """
    def __init__(self, map_data):
        """
        Construtor do A*.
        
        Args:
            map_data (np.array): Uma matriz 2D numpy representando o mapa.
        """
        self.map_data = map_data
        self.height, self.width = map_data.shape
        # A linha abaixo foi removida, pois esta classe não é um nó ROS e não tem um logger.
        # self.get_logger().info("Algoritmo A* inicializado com o mapa.")

    def heuristic(self, a, b):
        """
        Calcula a heurística (distância estimada) entre dois pontos.
        """
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def get_neighbors(self, node):
        """
        Obtém os vizinhos de um determinado nó na grade.
        """
        neighbors = []
        for dx, dy, cost in [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
                             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]:
            
            x, y = node[0] + dx, node[1] + dy

            if 0 <= x < self.width and 0 <= y < self.height:
                if self.map_data[y, x] <= 50:
                    neighbors.append(((x, y), cost))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """
        Reconstrói o caminho do final para o início.
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def find_path(self, start, goal):
        """
        Executa o algoritmo A* para encontrar o caminho mais curto.
        """
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
        
        # Se não encontrar caminho, retorna None silenciosamente.
        # O 'path_planner_node' é responsável por logar o aviso.
        return None
