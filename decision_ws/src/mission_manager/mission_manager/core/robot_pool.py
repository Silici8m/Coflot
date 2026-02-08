# robot_pool.py

import threading
from typing import Dict, Optional, Any

from rclpy.node import Node
from .robot import Robot
from fleet_interfaces.msg import RobotStateArray

class RobotPool:
    """
    Manages a collection of robots detected in the fleet.

    This class maintains a local registry of robots, handling their addition,
    removal, and state updates based on incoming fleet state messages. It provides
    access to robot instances and their adapters.

    Attributes:
        node (Node): Reference to the parent ROS 2 node.
        _robots (Dict[str, Robot]): Dictionary storing Robot instances indexed by robot ID.
    """

    def __init__(self, parent_node: Node) -> None:
        """
        Initializes the RobotPool.

        Args:
            parent_node (Node): The parent ROS 2 node.
        """
        self._robots: Dict[str, Robot] = {}
        self.node = parent_node
        
        self.logger = parent_node.get_logger()
        self._lock = threading.RLock()
    
    def is_robot_in_pool(self, robot_id: str) -> bool:
        """
        Checks if a robot is currently registered in the pool.

        Args:
            robot_id (str): The unique identifier of the robot.

        Returns:
            bool: True if the robot exists in the pool, False otherwise.
        """
        return robot_id in self._robots
        
    def _add_robot(self, robot_state_msg: Any) -> None:
        """
        Internal method to add a new robot to the pool.

        Args:
            robot_state_msg (Any): The state message containing initial robot data.

        Raises:
            ValueError: If the robot ID already exists in the pool.
        """
        with self._lock:
            if self.is_robot_in_pool(robot_state_msg.robot_id):
                raise ValueError(f"Robot {robot_state_msg.robot_id} already exists.")
            
            self._robots[robot_state_msg.robot_id] = Robot(self.node, robot_state_msg)
            
            
    def get_robot_adapter(self, robot_id: str) -> Optional[Any]:
        """
        Retrieves the adapter associated with a specific robot.

        Args:
            robot_id (str): The unique identifier of the robot.

        Returns:
            Optional[Any]: The robot's adapter instance, or None if the robot is not found.
        """
        with self._lock:
            robot = self._robots.get(robot_id, None)
            if robot is None:
                return None
            else:
                return robot.adapter 
        
    def get_robot(self, robot_id: str) -> Optional[Robot]:
        """
        Retrieves a Robot instance by its ID.

        Args:
            robot_id (str): The unique identifier of the robot.

        Returns:
            Optional[Robot]: The Robot instance, or None if not found.
        """
        with self._lock:
            return self._robots.get(robot_id, None)
        
    def get_all_robots(self) -> Dict[str, Robot]:
        """
        Retrieves the dictionary of all registered robots.

        Returns:
            Dict[str, Robot]: A dictionary mapping robot IDs to Robot instances.
        """
        with self._lock:
            return self._robots
        
    def update_pool(self, robot_state_array_msg: RobotStateArray) -> None:
        """
        Updates the pool based on the latest fleet state message.

        Adds new robots, updates existing ones, and removes robots that are no longer
        present in the received message.

        Args:
            robot_state_array_msg (RobotStateArray): The message containing states of all visible robots.
        """
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