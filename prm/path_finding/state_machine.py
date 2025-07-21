from enum import Enum, auto

class State(Enum):
    """
    Enumeração de todos os estados possíveis para a máquina de estados do robô.
    """
    # --- ESTADOS ATUALIZADOS ---
    NAVIGATING_TO_FLAG = auto() # NOVO ESTADO: Navega para a posição conhecida da bandeira.
    WAITING_FOR_PATH = auto()
    SEARCHING_FLAG = auto()     # Gira no local para encontrar a bandeira visualmente (usado para alinhamento fino).
    FOLLOWING_PATH = auto()     # Segue um caminho gerado pelo A*.
    ALIGNING_FOR_CAPTURE = auto() # Aproximação final da bandeira usando a visão.
    CAPTURING_FLAG = auto()     # Executa a sequência de captura com a garra.
    RETURNING_TO_BASE = auto()  # Define o alvo de retorno para a base.
    DEPOSITING_FLAG = auto()    # Executa a sequência de depósito com a garra.
    DONE = auto()               # Missão concluída, o robô para.
    

