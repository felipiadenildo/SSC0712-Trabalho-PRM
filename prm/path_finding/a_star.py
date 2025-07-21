# -*- coding: utf-8 -*-

# Nome do Arquivo: a_star.py
#
# Descrição:
# Contém uma implementação do algoritmo de busca de caminho A* ("A-Estrela").
# Esta classe é projetada para encontrar o caminho mais curto entre dois pontos em um
# grid 2D, levando em conta os obstáculos. Uma característica fundamental é a
# "inflação" dos obstáculos, que cria uma margem de segurança para garantir que
# o robô não passe muito perto das paredes ou outros perigos.
# Além disso, depois de encontrar o caminho bruto, fazemos uma suavização simples
# para reduzir zig‑zags indesejados.

import heapq
import numpy as np
import cv2  # Usado para a operação de dilatação, uma forma eficiente de inflar obstáculos
from typing import Optional, List, Tuple, Dict

# Define um tipo para representar um ponto no grid (coordenadas x, y)
Point = Tuple[int, int]
# Define um tipo para representar um caminho como uma lista de pontos
Path = List[Point]


class AStar:
    """
    Uma classe para encapsular o algoritmo de busca de caminho A*, com suporte
    integrado para inflação de obstáculos e suavização de trajetória.
    """

    def __init__(self, map_data: np.ndarray, safety_radius_cells: int = 3, smooth_window: int = 3):
        """
        Construtor da classe AStar.

        Args:
            map_data (np.ndarray): Matriz 2D numpy representando o mapa de ocupação.
                                   Valores maiores que 50 são considerados obstáculos.
            safety_radius_cells (int): Número de células (pixels) para inflar
                                       ao redor de cada obstáculo.
            smooth_window (int): Tamanho da janela para suavização da trajetória.
        """
        if map_data is None or map_data.size == 0:
            raise ValueError("O mapa fornecido não pode ser nulo ou vazio.")

        self.original_map = map_data
        self.height, self.width = map_data.shape
        self.safety_radius = safety_radius_cells
        self.smooth_window = smooth_window

        # Cria mapa inflacionado para planejamento
        self.inflated_map = self._inflate_obstacles()

    def _inflate_obstacles(self) -> np.ndarray:
        """
        Cria uma margem de segurança ao redor dos obstáculos no mapa usando dilatação morfológica.
        """
        obstacle_mask = np.where(self.original_map > 50, 255, 0).astype(np.uint8)
        kernel_size = 2 * self.safety_radius + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

        inflated_map = self.original_map.copy()
        inflated_map[inflated_mask == 255] = 100

        print(f"[A*] Mapa inflacionado com raio de segurança = {self.safety_radius} células.")
        return inflated_map

    def _heuristic(self, p1: Point, p2: Point) -> float:
        """Distância Euclidiana admissível para grid 8-direções."""
        return np.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def _get_neighbors(self, node: Point) -> List[Tuple[Point, float]]:
        """Retorna vizinhos válidos no mapa inflacionado, com custo (1 ou 1.414)."""
        nbrs = []
        moves = [
            (0, 1, 1.0), (0, -1, 1.0),
            (1, 0, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (1, -1, 1.414),
            (-1, 1, 1.414), (-1, -1, 1.414),
        ]
        for dx, dy, cost in moves:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < self.width and 0 <= y < self.height:
                if self.inflated_map[y, x] <= 50:
                    nbrs.append(((x, y), cost))
        return nbrs

    def _reconstruct_path(self, came_from: Dict[Point, Point], current: Point) -> Path:
        """Reconstrói o caminho desde o destino até a origem."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def _smooth_path(self, path: Path) -> Path:
        """
        Suaviza trajetórias removendo zig‑zags.
        Usa média móvel em janela de size = self.smooth_window.
        """
        if len(path) < 2 or self.smooth_window < 2:
            return path
        smoothed = []
        w = self.smooth_window
        for i in range(len(path)):
            xs = [path[j][0] for j in range(max(0, i-w), min(len(path), i+w+1))]
            ys = [path[j][1] for j in range(max(0, i-w), min(len(path), i+w+1))]
            avg_x = int(sum(xs) / len(xs))
            avg_y = int(sum(ys) / len(ys))
            smoothed.append((avg_x, avg_y))
        # remove duplicatas consecutivas
        result = [smoothed[0]]
        for p in smoothed[1:]:
            if p != result[-1]:
                result.append(p)
        return result

    def find_path(self, start: Point, goal: Point) -> Optional[Path]:
        """
        Executa A*: retorna lista de pontos ou None.
        Após encontrar, aplica suavização antes de devolver.
        """
        # verifica se start/goal não caem em obstáculo inflacionado
        if self.inflated_map[start[1], start[0]] > 50:
            print(f"[A*] Erro: start {start} em obstáculo.")
            return None
        if self.inflated_map[goal[1], goal[0]] > 50:
            print(f"[A*] Erro: goal {goal} em obstáculo.")
            return None

        open_set = [(0.0, 0.0, start)]
        heapq.heapify(open_set)
        came_from: Dict[Point, Point] = {}
        g_score: Dict[Point, float] = {start: 0.0}

        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            if current == goal:
                raw = self._reconstruct_path(came_from, current)
                print(f"[A*] Caminho encontrado: {len(raw)} pontos. Suavizando...")
                return self._smooth_path(raw)

            for neighbor, cost in self._get_neighbors(current):
                tentative_g = current_g + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        print("[A*] Aviso: nenhum caminho encontrado.")
        return None
