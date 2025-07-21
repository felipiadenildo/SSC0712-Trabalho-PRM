# -*- coding: utf-8 -*-

# Nome do Arquivo: path_planner_util.py
#
# Descrição:
# Este arquivo fornece uma coleção de funções utilitárias para planejamento e
# otimização de caminhos. Ele implementa uma abordagem customizada que não
# apenas encontra um caminho, mas também o otimiza para ser mais seguro (longe de
# obstáculos) e mais curto (removendo pontos desnecessários).
#
# Nota: Este módulo é uma alternativa ou complemento ao A* tradicional.
# Suas funções podem ser usadas para pós-processar um caminho gerado por A*
# ou como um planejador independente.

import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import binary_dilation # Importante: requer scipy

def convert_occupancy_grid_to_numpy(msg: OccupancyGrid) -> np.ndarray:
    """
    Converte uma mensagem OccupancyGrid em uma matriz 2D numpy.

    A conversão lida com a diferença de sistemas de coordenadas entre o ROS
    (origem no canto inferior esquerdo) e o Numpy (origem no canto superior esquerdo).

    Valores no mapa de retorno:
    - 0 a 100: Custo/Probabilidade de ocupação.
    - -1: Desconhecido.

    Args:
        msg: A mensagem nav_msgs.msg.OccupancyGrid recebida do tópico do mapa.

    Returns:
        Uma matriz numpy representando o mapa.
    """
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    
    # A especificação do OccupancyGrid define que a célula (0,0) está no canto
    # inferior esquerdo do mapa. Ao visualizar uma matriz numpy, o índice (0,0)
    # geralmente corresponde ao canto superior esquerdo. Inverter a matriz
    # verticalmente (np.flipud) alinha a representação numpy com a do ROS.
    return np.flipud(data)

def create_walkable_map(base_map: np.ndarray, safety_radius: int) -> np.ndarray:
    """
    Gera um "mapa de caminhabilidade" (walkable map) a partir de um mapa base,
    inflando os obstáculos para criar uma margem de segurança para o robô.

    Este processo é fundamental para garantir que o caminho planejado não apenas
    evite colisões, mas mantenha o robô a uma distância segura das paredes.
    Ele utiliza a operação de dilatação morfológica, que é altamente eficiente.

    Args:
        base_map: O mapa original (retornado por convert_occupancy_grid_to_numpy).
        safety_radius: O raio da margem de segurança ao redor dos obstáculos, em células do grid.

    Returns:
        Um mapa binário onde 1 representa uma área segura (caminhável) e 0
        representa uma área não segura (obstáculo ou muito perto de um).
    """
    # Considera células definitivamente ocupadas (valor > 50) e desconhecidas (-1) como obstáculos.
    # Esta é uma abordagem conservadora e segura.
    obstacle_mask = ((base_map > 50) | (base_map == -1))

    # Cria um "elemento estruturante" em forma de disco para a dilatação.
    # Este elemento define a forma da inflação ao redor de cada obstáculo.
    y, x = np.ogrid[-safety_radius:safety_radius+1, -safety_radius:safety_radius+1]
    structure_element = x**2 + y**2 <= safety_radius**2

    # A função binary_dilation "engorda" todas as áreas marcadas como True (obstáculos)
    # na máscara, usando o elemento estruturante. O resultado é um novo mapa onde
    # os obstáculos originais e a área de segurança ao redor deles são marcados como True.
    inflated_obstacles = binary_dilation(obstacle_mask, structure=structure_element)

    # O mapa de caminhabilidade é a inversão do mapa de obstáculos inflados.
    # Áreas seguras (False no mapa inflado) tornam-se 1 (caminhável), e
    # áreas perigosas (True) tornam-se 0 (não caminhável).
    walkable_map = (~inflated_obstacles).astype(np.uint8)
    
    return walkable_map


def check_line_of_sight(p1: tuple[int, int], p2: tuple[int, int], walkable_map: np.ndarray) -> bool:
    """
    Verifica se existe uma linha de visão direta e segura entre dois pontos.

    Usa um algoritmo similar ao de Bresenham para iterar sobre todas as células
    do grid que a linha reta entre p1 e p2 cruza, verificando se alguma delas
    é não-caminhável.

    Args:
        p1: Ponto de partida (linha, coluna).
        p2: Ponto de chegada (linha, coluna).
        walkable_map: O mapa binário de áreas caminháveis (1) e não caminháveis (0).

    Returns:
        True se a linha de visão estiver livre, False caso contrário.
    """
    x0, y0 = p1
    x1, y1 = p2
    
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    
    err = dx + dy
    
    while True:
        # Verifica se a célula atual é caminhável
        if walkable_map[x0, y0] == 0:
            return False # Obstáculo no caminho
            
        if x0 == x1 and y0 == y1:
            break # Chegou ao destino
            
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
            
    return True


def simplify_path(path: list[tuple[int, int]], walkable_map: np.ndarray) -> list[tuple[int, int]]:
    """
    Reduz o número de pontos em um caminho (weeding), removendo waypoints
    intermediários se houver uma linha de visão direta e segura entre os pontos.

    Args:
        path: A lista de pontos (waypoints) original.
        walkable_map: O mapa de áreas caminháveis.

    Returns:
        Uma nova lista de pontos representando o caminho simplificado.
    """
    if len(path) < 3:
        return path[:] # Retorna uma cópia do caminho se ele for muito curto para simplificar

    simplified_path = [path[0]]
    last_added_waypoint = path[0]

    for i in range(1, len(path) - 1):
        next_waypoint = path[i+1]
        
        # Se NÃO houver linha de visão direta do último ponto adicionado para o próximo...
        if not check_line_of_sight(last_added_waypoint, next_waypoint, walkable_map):
            # ...o ponto atual (path[i]) é necessário como um ponto de virada.
            simplified_path.append(path[i])
            last_added_waypoint = path[i]

    simplified_path.append(path[-1]) # Sempre adiciona o ponto final.
    return simplified_path

# As funções abaixo (`min_dist`, `shell_points`, `local_max_distance_point`, `adjust_path`)
# fazem parte de uma estratégia de otimização para tornar o caminho mais seguro,
# afastando-o das paredes. Elas podem ser computacionalmente intensivas e devem
# ser usadas com cuidado.

def min_dist_to_obstacle(point: tuple[int, int], walkable_map: np.ndarray, num_rays: int = 16, max_radius: int = 50) -> int:
    """
    Calcula a distância mínima de um ponto a um obstáculo, lançando "raios"
    em várias direções e medindo a distância até a primeira célula não-caminhável.

    Args:
        point: O ponto de referência (linha, coluna).
        walkable_map: O mapa de áreas caminháveis.
        num_rays: O número de raios a serem lançados. Mais raios = mais precisão, mais custo.
        max_radius: A distância máxima de busca.

    Returns:
        A menor distância encontrada até um obstáculo, em células.
    """
    h, w = walkable_map.shape
    angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
    min_d = max_radius
    
    for theta in angles:
        for r in range(1, max_radius):
            # Calcula as coordenadas do ponto ao longo do raio
            i = int(round(point[0] + r * np.sin(theta)))
            j = int(round(point[1] + r * np.cos(theta)))
            
            # Se o ponto sair do mapa ou for um obstáculo...
            if not (0 <= i < h and 0 <= j < w) or walkable_map[i, j] == 0:
                min_d = min(min_d, r) # Atualiza a distância mínima
                break # Passa para o próximo ângulo
                
    return min_d

def find_safest_point_locally(point: tuple[int, int], walkable_map: np.ndarray, search_kernel_size: int = 2) -> tuple[int, int]:
    """
    Encontra o ponto mais seguro (mais distante de obstáculos) em uma vizinhança local.

    Args:
        point: O ponto original (linha, coluna).
        walkable_map: O mapa de áreas caminháveis.
        search_kernel_size: O raio da vizinhança a ser pesquisada (em células).

    Returns:
        O ponto mais seguro encontrado na vizinhança.
    """
    h, w = walkable_map.shape
    i, j = point
    k = search_kernel_size
    
    # Define a região de busca (bounding box) ao redor do ponto.
    x1, x2 = max(0, i - k), min(h, i + k + 1)
    y1, y2 = max(0, j - k), min(w, j + k + 1)
    
    # Coleta todos os pontos caminháveis dentro da região.
    local_region = [(row, col) for row in range(x1, x2) for col in range(y1, y2) if walkable_map[row, col] == 1]
    
    if not local_region:
        return point # Se não houver pontos caminháveis, retorna o original.
        
    # Usa a função `min_dist_to_obstacle` para encontrar o ponto na região
    # que maximiza a distância para o obstáculo mais próximo.
    safest_point = max(local_region, key=lambda p: min_dist_to_obstacle(p, walkable_map))
    return safest_point

def adjust_path_for_safety(path: list[tuple[int, int]], walkable_map: np.ndarray, search_kernel_size: int = 2) -> list[tuple[int, int]]:
    """
    Aplica a otimização de segurança a todos os pontos do caminho, movendo-os
    ligeiramente para posições localmente mais seguras.

    Args:
        path: A lista de pontos do caminho.
        walkable_map: O mapa de áreas caminháveis.
        search_kernel_size: O raio da vizinhança para a busca de segurança.

    Returns:
        Um novo caminho com os pontos ajustados para maior segurança.
    """
    return [find_safest_point_locally(pt, walkable_map, search_kernel_size) for pt in path]