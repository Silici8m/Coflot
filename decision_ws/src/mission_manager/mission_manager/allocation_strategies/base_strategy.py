# base_strategy.py

from abc import ABC, abstractmethod
from typing import List, Optional
from rclpy.node import Node

from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry
from mission_manager.core.mission import Mission, MissionState
from .allocation_interface import AllocationDecision, AllocationAction

class AllocatorStrategy(ABC):
    """
    Abstract base class for mission allocation strategies.

    This class defines the interface that all specific allocation algorithms
    (e.g., Auction-based, Greedy, Hungarian) must implement. It provides
    access to the system state via the robot pool and mission registry.

    Attributes:
        node (Node): Reference to the parent ROS 2 node.
        robot_pool (RobotPool): Reference to the robot pool manager.
        registry (MissionRegistry): Reference to the mission registry.
    """
    
    def __init__(self, node: Node, robot_pool: RobotPool, mission_registry: MissionRegistry) -> None:
        """
        Initializes the AllocatorStrategy.

        Args:
            node (Node): The parent ROS 2 node.
            robot_pool (RobotPool): The robot pool manager instance.
            mission_registry (MissionRegistry): The mission registry instance.
        """
        self.node = node
        self.robot_pool = robot_pool
        self.registry = mission_registry
        self.logger = self.node.get_logger()

    @abstractmethod
    def allocate(self) -> List[AllocationDecision]:
        """
        Analyzes the current state and computes a list of allocation decisions.

        This method must be implemented by subclasses to define the specific
        allocation logic (matching robots to missions).

        Returns:
            List[AllocationDecision]: A list of decisions to be executed by the dispatcher.
        """
        pass

    def _get_required_action(self, current_mission: Optional[Mission]) -> AllocationAction:
        """
        Determines the technical action required based on the robot's current mission state.

        This method purely evaluates the mechanical state transition required (e.g.,
        revoking, suspending) without judging the feasibility or cost of the action.

        Args:
            current_mission (Optional[Mission]): The mission currently assigned to the robot,
                                                 or None if the robot is free.

        Returns:
            AllocationAction: The specific action type required.
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