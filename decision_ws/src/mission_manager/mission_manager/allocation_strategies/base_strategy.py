from abc import ABC, abstractmethod
from typing import List
from rclpy.node import Node

# On importe uniquement pour le type hinting (pas d'instanciation ici)
from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry
from .allocation_interface import AllocationDecision

class AllocatorStrategy(ABC):
    
    def __init__(self, node: Node, robot_pool: RobotPool, mission_registry: MissionRegistry):
        self.node = node
        self.robot_pool = robot_pool
        self.registry = mission_registry

    @abstractmethod
    def allocate(self) -> List[AllocationDecision]:
        """
        Analyse l'état actuel (via self.robot_pool et self.registry)
        et retourne une liste de décisions à exécuter.
        """
        pass