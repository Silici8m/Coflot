from enum import Enum
from dataclasses import dataclass
from typing import List

class AllocationAction(Enum):
    """
    Définit le TYPE d'action à effectuer.
    Cela permet au Dispatcher d'appliquer la bonne procédure sans deviner.
    """
    START = 1               # Démarrage standard (Robot libre -> Mission)
    CANCEL_AND_START = 2    # Préemption (Urgence) : On annule la mission en cours, on lance la nouvelle
    SUSPEND_AND_START = 3   # Suspension : On met en pause la mission en cours, on lance la nouvelle
    # WAIT = 4              # (Optionnel) Expliciter qu'on attend (ex: charge batterie)

@dataclass
class AllocationDecision:
    """
    Objet de transfert de données (DTO) représentant une décision de l'allocateur.
    """
    action: AllocationAction
    mission_id: str
    robot_id: str
    reason: str = ""