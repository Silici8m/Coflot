# allocation_interface.py

from enum import Enum
from dataclasses import dataclass

class AllocationAction(Enum):
    """
    Définit le TYPE d'action à effectuer.
    Cela permet au Dispatcher d'appliquer la bonne procédure sans deviner.
    """
    ASSIGN_AND_START = 1               # Démarrage standard (Robot libre -> Mission)
    REVOKE = 2    # Préemption (Urgence) : On annule la mission en cours, on lance la nouvelle
    SUSPEND = 3   # Suspension : On met en pause la mission en cours, on lance la nouvelle
    NOTHING = 4

@dataclass
class AllocationDecision:
    """
    Objet de transfert de données (DTO) représentant une décision de l'allocateur.
    """
    action: AllocationAction
    mission_id: str
    robot_id: str
    reason: str = ""