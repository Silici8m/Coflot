# mission_registry.py

import threading
from fleet_interfaces.msg import MissionRequest 

from .mission import Mission, MissionState
from .robot_pool import RobotPool
from mission_manager.config import CLEAR_FINISHED_PERIOD

class MissionRegistry:
    def __init__(self, parent_node, robot_pool: RobotPool):
        self._missions = {}
        self.node = parent_node
        self.robot_pool = robot_pool
        
        self.clear_finished_period = CLEAR_FINISHED_PERIOD
        self.auto_delete_finished = self.node.create_timer(
                self.clear_finished_period, 
                self._clear_finished_missions
            )
        
        self.logger = parent_node.get_logger()
        self._lock = threading.RLock()
        
    def add_mission(self, mission_request : MissionRequest) -> None:
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
        
    
    def get_mission(self, mission_id) -> Mission | None:
        with self._lock:
            return self._missions.get(mission_id, None)
    
    
    def _remove_mission(self, mission_id):
        with self._lock:
            return self._missions.pop(mission_id, None)
            
    
    def get_mission_by_robot(self, robot_id) -> Mission:
        with self._lock:
            for mission in self._missions.values():
                if mission.assigned_robot_id == robot_id:
                    return mission
            return None
    
        
    def handle_validation(self, mission_id):
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.on_validation()
        else :
            self.logger.error(f"Mission {mission_id} not found for validation.")
            
    def mission_assign(self, mission_id, robot_id):
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
            self.logger.info(f"Assigned robot {robot_id} to mission {mission_id}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to assign {robot_id} to {mission_id}: {e}")
            return False
        
        
    def mission_suspend(self, mission_id):
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.suspend()
        else :
            self.logger.error(f"Mission {mission_id} not found for suspension.")
            
    def mission_start_approaching(self, mission_id):
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.start_approaching()
        else :
            self.logger.error(f"Mission {mission_id} not found for starting approaching.")
            
    def mission_revoke(self, mission_id):
        mission = self.get_mission(mission_id)
        if mission is not None:
            mission.revoke()
        else :
            self.logger.error(f"Mission {mission_id} not found for revocation.")
    
    def get_missions_by_state(self, state):
        with self._lock:
            return [mission for mission in self._missions.values() if mission.state == state]
    
    def get_missions_list(self):
        with self._lock:
            return list(self._missions.values())
        
    def get_mission_dict(self):
        with self._lock:
            return self._missions
        
    def _clear_finished_missions(self):
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
                    self.logger.info(f"Cleaned up finished/failed mission: {mid}")       