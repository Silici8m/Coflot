# mission_registry.py

import threading
from typing import Dict, List, Optional

from rclpy.node import Node
from rclpy.timer import Timer
from fleet_interfaces.msg import MissionRequest 

from .mission import Mission, MissionState
from .robot_pool import RobotPool
from mission_manager.config import CLEAR_FINISHED_PERIOD

class MissionRegistry:
    """
    Central registry managing the lifecycle and storage of all missions.

    This class acts as a thread-safe database for missions. It handles creation,
    retrieval, assignment coordination, and automatic cleanup of finished missions.
    It serves as the interface between the high-level node logic and individual Mission objects.

    Attributes:
        node (Node): Reference to the parent ROS 2 node.
        robot_pool (RobotPool): Reference to the robot pool manager.
        _missions (Dict[str, Mission]): Dictionary storing active and pending missions, indexed by ID.
    """

    def __init__(self, parent_node: Node, robot_pool: RobotPool) -> None:
        """
        Initializes the MissionRegistry.

        Args:
            parent_node (Node): The ROS 2 node that owns this registry.
            robot_pool (RobotPool): The robot pool manager instance.
        """
        self._missions: Dict[str, Mission] = {}
        self.node = parent_node
        self.robot_pool = robot_pool
        
        self.auto_delete_finished: Timer = self.node.create_timer(
                CLEAR_FINISHED_PERIOD, 
                self._clear_finished_missions
            )
        
        self.logger = parent_node.get_logger()
        self._lock = threading.RLock()
        
    def add_mission(self, mission_request: MissionRequest) -> None:
        """
        Creates and registers a new Mission based on a request message.

        If a mission with the same ID already exists, the creation is skipped
        and an error is logged.

        Args:
            mission_request (MissionRequest): The message containing mission details (ID, priority, waypoints).
        """
        with self._lock:
            if mission_request.mission_id in self._missions:
                self.logger.error(f"Trying to add {mission_request.mission_id}. Mission ID {mission_request.mission_id} already exists.")
                return
            
            self._missions[mission_request.mission_id] = Mission(
                parent_node=self.node,
                mission_id=mission_request.mission_id,
                priority=mission_request.priority,
                waypoints=mission_request.waypoints,
                request_stamp=mission_request.header.stamp
            )
        self.logger.info(f"Mission {mission_request.mission_id} added.")
        
    
    def get_mission(self, mission_id: str) -> Optional[Mission]:
        """
        Retrieves a mission by its unique identifier.

        Args:
            mission_id (str): The unique identifier of the mission.

        Returns:
            Optional[Mission]: The mission object if found, None otherwise.
        """
        with self._lock:
            return self._missions.get(mission_id, None)
    
    
    def _remove_mission(self, mission_id: str) -> Optional[Mission]:
        """
        Removes a specific mission from the registry.

        Args:
            mission_id (str): The ID of the mission to remove.

        Returns:
            Optional[Mission]: The removed mission object, or None if it did not exist.
        """
        with self._lock:
            return self._missions.pop(mission_id, None)
            
    
    def get_mission_by_robot(self, robot_id: str) -> Optional[Mission]:
        """
        Finds the mission currently assigned to a specific robot.

        Note:
            This performs a linear search over all missions.

        Args:
            robot_id (str): The unique identifier of the robot.

        Returns:
            Optional[Mission]: The active mission assigned to the robot, or None if not found.
        """
        with self._lock:
            for mission in self._missions.values():
                if mission.assigned_robot_id == robot_id:
                    return mission
            return None
    
        
    def handle_validation(self, mission_id: str) -> None:
        """
        Triggers the validation event for a specific mission.

        This is typically called when a human operator validates a step.

        Args:
            mission_id (str): The unique identifier of the mission.
        """
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.on_validation()
        else :
            self.logger.error(f"Mission {mission_id} not found for validation.")
            
    def mission_assign(self, mission_id: str, robot_id: str) -> bool:
        """
        Attempts to assign a robot to a mission.

        Retrieves the mission and the robot adapter, then performs the assignment.

        Args:
            mission_id (str): The ID of the mission to assign.
            robot_id (str): The ID of the robot to assign.

        Returns:
            bool: True if assignment was successful, False otherwise (e.g., mission/robot not found or busy).
        """
        mission = self.get_mission(mission_id)
        robot_adapter = self.robot_pool.get_robot_adapter(robot_id)
        
        if mission is None:
            self.logger.error(f"Assign failed: Mission {mission_id} not found.")
            return False
        if robot_adapter is None:
            self.logger.error(f"Assign failed: Robot {robot_id} not found in pool.")
            return False
        
        try:
            mission.assign(robot_id, robot_adapter)
            #self.logger.info(f"Assigned robot {robot_id} to mission {mission_id}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to assign {robot_id} to {mission_id}: {e}")
            return False
        
        
    def mission_suspend(self, mission_id: str) -> None:
        """
        Requests suspension of a specific mission.

        Args:
            mission_id (str): The unique identifier of the mission.
        """
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.suspend()
        else :
            self.logger.error(f"Mission {mission_id} not found for suspension.")
            
    def mission_start_approaching(self, mission_id: str) -> None:
        """
        Commands a mission to start the approaching phase.

        Args:
            mission_id (str): The unique identifier of the mission.
        """
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.start_approaching()
        else :
            self.logger.error(f"Mission {mission_id} not found for starting approaching.")
            
    def mission_revoke(self, mission_id: str) -> None:
        """
        Revokes a mission, effectively cancelling its current assignment.

        Args:
            mission_id (str): The unique identifier of the mission.
        """
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.revoke()
        else :
            self.logger.error(f"Mission {mission_id} not found for revocation.")
    
    def get_missions_by_state(self, state: MissionState) -> List[Mission]:
        """
        Retrieves a list of all missions currently in a specific state.

        Args:
            state (MissionState): The state to filter by.

        Returns:
            List[Mission]: A list of matching Mission objects.
        """
        with self._lock:
            return [mission for mission in self._missions.values() if mission.state == state]
    
    def get_missions_list(self) -> List[Mission]:
        """
        Retrieves all registered missions as a list.

        Returns:
            List[Mission]: A list containing all managed Mission objects.
        """
        with self._lock:
            return list(self._missions.values())
        
    def get_mission_dict(self) -> Dict[str, Mission]:
        """
        Retrieves the internal dictionary of missions.

        Warning:
            Returns a direct reference to the internal dictionary. External modification
            is not thread-safe.

        Returns:
            Dict[str, Mission]: The dictionary mapping mission IDs to Mission objects.
        """
        with self._lock:
            return self._missions
        
    def _clear_finished_missions(self) -> None:
        """
        Timer callback to clean up finished or failed missions.

        Iterates through the registry and removes missions that have reached
        a terminal state (FINISHED or FAILED) to prevent memory leaks.
        """
        with self._lock:
            # 1. Identifier les missions à supprimer
            # On crée une liste temporaire pour ne pas itérer sur le dict qu'on modifie
            ids_to_remove = [
                m.mission_id 
                for m in self._missions.values() 
                if m.state in [MissionState.FINISHED, MissionState.FAILED]
            ]
            
            # 2. Supprimer les missions identifiées
            if ids_to_remove:
                for mid in ids_to_remove:
                    self._remove_mission(mid)
                    #self.logger.info(f"Cleaned up finished/failed mission: {mid}")