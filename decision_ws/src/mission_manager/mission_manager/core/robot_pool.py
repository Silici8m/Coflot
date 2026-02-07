# robot_pool.py

import threading

from .robot import Robot
from fleet_interfaces.msg import RobotStateArray

class RobotPool:
    def __init__(self, parent_node):
        self._robots = {}
        self.node = parent_node
        
        self.logger = parent_node.get_logger()
        self._lock = threading.RLock()
    
    def is_robot_in_pool(self, robot_id):
        return robot_id in self._robots
        
    def _add_robot(self, robot_state_msg):
        with self._lock:
            if self.is_robot_in_pool(robot_state_msg.robot_id):
                raise ValueError(f"Robot {robot_state_msg.robot_id} already exists.")
            
            self._robots[robot_state_msg.robot_id] = Robot(self.node, robot_state_msg)
            
            
    def get_robot_adapter(self, robot_id):
        with self._lock:
            robot = self._robots.get(robot_id, None)
            if robot is None:
                return None
            else:
                return robot.adapter 
        
    def get_robot(self, robot_id):
        with self._lock:
            return self._robots.get(robot_id, None)
        
    def get_all_robots(self):
        with self._lock:
            return self._robots
        
    def update_pool(self, robot_state_array_msg : RobotStateArray):
        # Fonction qui sera appelée a chaque callback du subscriber /fleet/fleet_state
        with self._lock:
            received_robot_ids = set()
            
            for robot_state_msg in robot_state_array_msg.robots:
                r_id = robot_state_msg.robot_id
                received_robot_ids.add(r_id)
                
                if not self.is_robot_in_pool(r_id):
                    self._add_robot(robot_state_msg)
                    self.logger.info(f"Robot {r_id} added to pool.")
                
                self._robots[r_id].update_state(robot_state_msg)
            
            local_robot_ids = set(self._robots.keys())
            disappeared_robots = local_robot_ids - received_robot_ids
            
            for robot_id in disappeared_robots:
                self.logger.info(f"Robot {robot_id} removed from pool.")
                self._robots[robot_id].adapter.destroy()
                del self._robots[robot_id]