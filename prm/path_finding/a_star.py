# -*- coding: utf-8 -*-

# Nome do Arquivo: a_star.py
#
# Descrição:
# Contém uma implementação do algoritmo de busca de caminho A* ("A-Estrela").
# Esta classe é projetada para encontrar o caminho mais curto entre dois pontos em um
# grid 2D, levando em conta os obstáculos. Uma característica fundamental é a
# "inflação" dos obstáculos, que cria uma margem de segurança para garantir que
# o robô não passe muito perto das paredes ou outros perigos.

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
    integrado para inflação de obstáculos para garantir a segurança da navegação.
    """

    def __init__(self, map_data: np.ndarray, safety_radius_cells: int = 3):
        """
        Construtor da classe AStar.

        Args:
            map_data (np.ndarray): Uma matriz 2D numpy representando o mapa de ocupação.
                                   Valores maiores que 50 são considerados obstáculos.
            safety_radius_cells (int): O número de células (píxeis) para inflar
                                       ao redor de cada obstáculo, criando a margem de segurança.
        """
        if map_data is None or map_data.size == 0:
            raise ValueError("O mapa fornecido não pode ser nulo ou vazio.")

        self.original_map = map_data
        self.height, self.width = map_data.shape
        self.safety_radius = safety_radius_cells

        # O planejamento do caminho será feito em um mapa "inflacionado", não no original.
        # Isso garante que o caminho gerado já respeite as margens de segurança.
        self.inflated_map = self._inflate_obstacles()

    def _inflate_obstacles(self) -> np.ndarray:
        """
        Cria uma margem de segurança ao redor dos obstáculos no mapa usando dilatação morfológica.

        Returns:
            np.ndarray: Um novo mapa onde os obstáculos foram "engordados" pelo raio de segurança.
        """
        # 1. Cria uma máscara binária: 255 para obstáculos, 0 para espaço livre.
        #    O valor 50 é um limiar comum em mapas de ocupação ROS para definir um obstáculo.
        #    OpenCV requer o tipo `uint8` para suas operações de imagem.
        obstacle_mask = np.where(self.original_map > 50, 255, 0).astype(np.uint8)

        # 2. Define o "kernel", que é a forma da inflação.
        #    Um kernel quadrado de (2*r+1) x (2*r+1) cria uma margem de 'r' células
        #    em todas as 8 direções ao redor de cada pixel de obstáculo.
        kernel_size = 2 * self.safety_radius + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        # 3. Executa a dilatação. Esta função do OpenCV é extremamente otimizada.
        #    Ela expande as áreas brancas (nossos obstáculos) da máscara.
        inflated_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

        # 4. Cria uma cópia do mapa original e aplica a máscara inflacionada.
        #    Onde a máscara inflada for 255, o mapa receberá o valor 100 (obstáculo definitivo).
        inflated_map = self.original_map.copy()
        inflated_map[inflated_mask == 255] = 100

        print(f"[A*] Mapa inflacionado com um raio de segurança de {self.safety_radius} células.")
        return inflated_map

    def _heuristic(self, point1: Point, point2: Point) -> float:
        """
        Calcula a heurística de distância (custo estimado) entre dois pontos.
        Usa a Distância Euclidiana, que é admissível para um grid de 8 direções,
        garantindo que o A* encontre o caminho mais curto.

        Args:
            point1 (Point): O primeiro ponto (x, y).
            point2 (Point): O segundo ponto (x, y).

        Returns:
            float: A distância Euclidiana entre os dois pontos.
        """
        return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def _get_neighbors(self, node: Point) -> List[Tuple[Point, float]]:
        """
        Obtém os vizinhos válidos de um nó no grid, verificando o MAPA INFLACIONADO.

        Args:
            node (Point): O ponto (x, y) para o qual encontrar os vizinhos.

        Returns:
            List[Tuple[Point, float]]: Uma lista de tuplas, onde cada tupla contém
                                       um vizinho válido e o custo do movimento para chegar até ele.
        """
        neighbors = []
        # Define os 8 possíveis movimentos (cardinais e diagonais) e seus custos.
        # Custo 1 para movimentos retos e sqrt(2) (~1.414) para diagonais.
        for dx, dy, cost in [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]:
            
            x, y = node[0] + dx, node[1] + dy

            # Verifica se o vizinho está dentro dos limites do mapa
            if 0 <= x < self.width and 0 <= y < self.height:
                # A verificação de obstáculo é feita no mapa com a margem de segurança!
                if self.inflated_map[y, x] <= 50:
                    neighbors.append(((x, y), cost))
        return neighbors

    def _reconstruct_path(self, came_from: Dict[Point, Point], current: Point) -> Path:
        """
        Reconstrói o caminho final, retrocedendo do nó de destino até o inicial.

        Args:
            came_from (Dict[Point, Point]): Dicionário que mapeia cada nó ao seu predecessor no caminho.
            current (Point): O nó de destino a partir do qual começar a reconstrução.

        Returns:
            Path: O caminho final como uma lista de pontos do início ao fim.
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        # O caminho é construído do fim para o início, então o invertemos antes de retornar.
        return total_path[::-1]

    def find_path(self, start: Point, goal: Point) -> Optional[Path]:
        """
        Executa o algoritmo A* para encontrar um caminho do início ao fim.

        Args:
            start (Point): As coordenadas (x, y) do ponto de partida.
            goal (Point): As coordenadas (x, y) do ponto de destino.

        Returns:
            Optional[Path]: Uma lista de pontos (o caminho) se encontrado, ou None caso contrário.
        """
        # --- Verificação de Validade Inicial ---
        # Garante que os pontos de início e fim não estejam dentro de um obstáculo inflado.
        if self.inflated_map[start[1], start[0]] > 50:
            print(f"[A*] Erro: O ponto de partida {start} está em uma área de obstáculo ou segurança.")
            return None
        if self.inflated_map[goal[1], goal[0]] > 50:
            print(f"[A*] Erro: O ponto de destino {goal} está em uma área de obstáculo ou segurança.")
            return None

        # --- Inicialização do A* ---
        # open_set é uma fila de prioridade (heap) para os nós a serem visitados.
        # Estrutura da tupla: (f_score, g_score, node). O g_score é usado para desempate,
        # favorecendo caminhos mais curtos quando os f_scores são iguais.
        open_set = [(0, 0, start)]
        heapq.heapify(open_set)

        # came_from armazena o nó predecessor para reconstruir o caminho.
        came_from: Dict[Point, Point] = {}

        # g_score armazena o custo do caminho mais barato conhecido do início até um nó.
        g_score: Dict[Point, float] = {start: 0}

        while open_set:
            # Obtém o nó na fila de prioridade com o menor f_score.
            _, current_g_score, current_node = heapq.heappop(open_set)

            # Se o nó atual é o destino, reconstruímos e retornamos o caminho.
            if current_node == goal:
                print(f"[A*] Caminho encontrado com sucesso!")
                return self._reconstruct_path(came_from, current_node)

            # Explora os vizinhos do nó atual.
            for neighbor, move_cost in self._get_neighbors(current_node):
                # O custo para chegar a este vizinho a partir do início.
                tentative_g_score = current_g_score + move_cost

                # Se este caminho para o vizinho é melhor que qualquer um já registrado...
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # ...registra este novo caminho como o melhor.
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
        
        # Se a fila de prioridade esvaziar e o destino não for alcançado, não há caminho.
        print("[A*] Aviso: Não foi possível encontrar um caminho.")
        return None