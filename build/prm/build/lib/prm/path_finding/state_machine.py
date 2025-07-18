# prm/path_finding/state_machine.py
from enum import Enum

class State(Enum):
    SEARCHING_FLAG = 1      # Procurando a bandeira (girando)
    MOVING_TO_FLAG = 2      # Estado transitório que requisita o caminho para a bandeira
    RETURNING_TO_BASE = 3   # Estado transitório que requisita o caminho para a base
    FOLLOWING_PATH = 4      # Seguindo a lista de waypoints recebida do planejador
    ALIGNING_FOR_CAPTURE = 5 # Alinhamento final para a captura
    CAPTURING_FLAG = 6      # Acionamento da garra para capturar
    DEPOSITING_FLAG = 7     # Acionamento da garra para soltar a bandeira
    DONE = 8                # Missão concluída