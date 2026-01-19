# robot.py

from .robot_adapter import RobotAdapter
from geometry_msgs.msg import Pose
from fleet_interfaces.msg import RobotState

class Robot:
    def __init__(self, node, state_msg):
        self.robot_id = state_msg.robot_id
        
        self.adapter = RobotAdapter(node, state_msg.robot_id)
        
        self._state_msg = state_msg

    def update_state(self, state_msg):
        self._state_msg = state_msg
        
    def get_position(self):
        return self._state_msg.pose
    
    def get_battery_pct(self):
        return self._state_msg.battery_pct
