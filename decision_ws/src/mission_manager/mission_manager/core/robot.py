# robot.py

from typing import Any
from rclpy.node import Node
from .robot_adapter import RobotAdapter

class Robot:
    """
    Represents a managed robot within the fleet.

    This class serves as a state container and control interface for a single robot.
    It holds the latest known state (position, battery) and maintains an instance
    of the RobotAdapter to handle navigation commands.

    Attributes:
        robot_id (str): The unique identifier of the robot.
        adapter (RobotAdapter): The interface used to send commands to the robot.
        _state_msg (Any): The latest state message received from the robot/fleet.
    """

    def __init__(self, node: Node, state_msg: Any) -> None:
        """
        Initializes the Robot instance.

        Args:
            node (Node): The parent ROS 2 node used to create the adapter.
            state_msg (Any): The initial state message containing robot_id, pose, and battery.
        """
        self.robot_id = state_msg.robot_id
        
        self.adapter = RobotAdapter(node, state_msg.robot_id)
        
        self._state_msg = state_msg

    def update_state(self, state_msg: Any) -> None:
        """
        Updates the cached state of the robot with a new message.

        Args:
            state_msg (Any): The new state message containing updated telemetry.
        """
        self._state_msg = state_msg
        
    def get_position(self) -> Any:
        """
        Retrieves the current position of the robot.

        Returns:
            Any: The pose object (typically geometry_msgs/Pose) from the state message.
        """
        return self._state_msg.pose
    
    def get_battery_pct(self) -> float:
        """
        Retrieves the current battery level of the robot.

        Returns:
            float: The battery percentage.
        """
        return self._state_msg.battery_pct