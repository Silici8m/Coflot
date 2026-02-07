# base_strategy.py

from abc import ABC, abstractmethod
from typing import List
from rclpy.node import Node

from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry
from mission_manager.core.mission import Mission, MissionState
from .allocation_interface import AllocationDecision, AllocationAction

class AllocatorStrategy(ABC):
    
    def __init__(self, node: Node, robot_pool: RobotPool, mission_registry: MissionRegistry):
        self.node = node
        self.robot_pool = robot_pool
        self.registry = mission_registry
        self.logger = self.node.get_logger()

    @abstractmethod
    def allocate(self) -> List[AllocationDecision]:
        """
        Analyse l'état actuel (via self.robot_pool et self.registry)
        et retourne une liste de décisions à exécuter.
        """
        pass

    def _get_required_action(self, current_mission: Mission) -> AllocationAction:
        """
        Détermine l'action TECHNIQUE requise en fonction de l'état du robot.
        Ne juge pas de la faisabilité (coût), juste de la mécanique.
        """
        # Robot Libre -> ASSIGN_AND_START
        if current_mission is None:
            return AllocationAction.ASSIGN_AND_START
            
        state = current_mission.state
        
        # Robot Libre -> ASSIGN_AND_START
        if state in [MissionState.FINISHED, MissionState.FAILED, MissionState.PENDING]:
            return AllocationAction.ASSIGN_AND_START
            
        # Robot En Approche -> REVOKE
        if state == MissionState.APPROACHING:
            return AllocationAction.REVOKE
            
        # Robot En Mission -> SUSPEND
        if state in [MissionState.WAITING, MissionState.DELIVERING]:
            return AllocationAction.SUSPEND
            
        # Robot Indisponible (Transition en cours) -> NOTHING  |  (SUSPENDING, DISCHARGING, ASSIGNED)
        return AllocationAction.NOTHING