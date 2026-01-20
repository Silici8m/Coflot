# mission.py

from enum import Enum
import threading

import rclpy
from rclpy.node import Node

from mission_manager.config import VALIDATION_TIMEOUT

class MissionState(Enum):
    PENDING = 0
    ASSIGNED = 1
    APPROACHING = 2
    WAITING = 3
    DELIVERING = 4
    SUSPENDING = 5
    DISCHARGING = 6
    FINISHED = 7
    FAILED = 8
    
    
class TransitionException(Exception):
    pass
    
    
class Mission:
    node: Node
    mission_id: str
    priority: int
    state: MissionState
    waypoints: list
    assigned_robot_id: str
    goal_waypoint_idx: int
    request_stamp: rclpy.time.Time
    #client: RobotNavigationActionClient
    timeout_timer: rclpy.timer.Timer

    def __init__(self, parent_node : Node, mission_id, priority, waypoints, request_stamp):
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.mission_id = mission_id
        self.priority = priority
        self.state = MissionState.PENDING
        self.waypoints = waypoints
        self.assigned_robot_id = None
        self.goal_waypoint_idx = None
        self.request_stamp = request_stamp
        self.robot_adapter = None
        self.timeout_timer = None
        self.deletion_finished_timeout = VALIDATION_TIMEOUT
        self.lock = threading.RLock()
        
        
    def assign(self, robot_id, robot_adapter):
        with self.lock:
            if self.state != MissionState.PENDING:
                raise TransitionException("Mission can only be assigned from PENDING state.")
            
            if robot_adapter.is_busy():
                raise Exception("Cannot assign mission to a robot whose action server is busy.")
            
            self.assigned_robot_id = robot_id
            self.robot_adapter = robot_adapter
            if self.goal_waypoint_idx == None:
                self.goal_waypoint_idx = 0
            self.state = MissionState.ASSIGNED
        
        
    def start_approaching(self):
        with self.lock:
            if self.state != MissionState.ASSIGNED:
                raise TransitionException("Mission can only start approaching from ASSIGNED state.")
            
            self.robot_adapter.send_goal(
                self.waypoints[self.goal_waypoint_idx],
                on_success=self.on_arrival_cb,
                on_failure=self.on_failure_cb
            )
            self.state = MissionState.APPROACHING
            
            
    def revoke(self):
        with self.lock:
            if self.state !=  MissionState.APPROACHING:
                raise TransitionException("Mission can only be revoked from APPROACHING state.")
            
            self._release_robot(cancel_active_goal=True)
            self.state = MissionState.PENDING


    def proceed(self):
        with self.lock:
            if self.state != MissionState.WAITING:
                raise TransitionException("Mission can only proceed from WAITING state.")
            
            self.goal_waypoint_idx += 1
            
            if self.goal_waypoint_idx >= len(self.waypoints):
                 raise Exception("Cannot proceed: Waypoint index out of bounds.")
            
            self.robot_adapter.send_goal(
                self.waypoints[self.goal_waypoint_idx],
                on_success=self.on_arrival_cb,
                on_failure=self.on_failure_cb
            )
            self.state = MissionState.DELIVERING
    
    
    def suspend(self):
        with self.lock:
            if self.state not in [MissionState.DELIVERING, MissionState.WAITING]:
                raise TransitionException("Mission can only be suspended from DELIVERING or WAITING state.")
            
            if self.goal_waypoint_idx == len(self.waypoints) - 1:
                self.logger.warning(f"Mission {self.mission_id}: Cannot suspend on last leg, finishing mission.")
                return  # No suspension at last waypoint
        
            match self.state:
                case MissionState.DELIVERING:
                    self.state = MissionState.SUSPENDING
                case MissionState.WAITING:
                    self.destroy_validation_timer()
                    self.state = MissionState.SUSPENDING
                    self.on_arrival_cb()


    def release(self):
        with self.lock:
            if self.state != MissionState.DISCHARGING:
                raise TransitionException("Mission can only be released from DISCHARGING state.")
            
            self._release_robot(cancel_active_goal=False)
            self.state = MissionState.PENDING
            self.logger.info(f"Mission {self.mission_id} released. Resumable at waypoint {self.goal_waypoint_idx}.")
            
            
    def complete(self):
        with self.lock:
            if self.state != MissionState.DISCHARGING:
                raise TransitionException("Mission can only be completed from DISCHARGING state.")
            
            self._release_robot(cancel_active_goal=False)
            self.state = MissionState.FINISHED
            self.logger.info(f"Mission {self.mission_id} finished successfully.")

    
    def _release_robot(self, cancel_active_goal):
        with self.lock:
            if self.robot_adapter is not None:
                if cancel_active_goal and self.robot_adapter.is_navigating():
                    self.robot_adapter.cancel_goal()
                    
                self.assigned_robot_id = None
                self.robot_adapter = None


    def on_arrival_cb(self):
        with self.lock:
            if self.state not in [MissionState.APPROACHING, MissionState.DELIVERING, MissionState.SUSPENDING]:
                raise TransitionException("Arrival event not valid in current state.")
            
            self.start_validation_timer()
            
            match self.state:
                case MissionState.APPROACHING:
                    # Wait until the robot is loaded
                    self.state = MissionState.WAITING
                
                case MissionState.SUSPENDING:
                    # Wait until the robot is unloaded
                    if self.goal_waypoint_idx == len(self.waypoints) - 1:
                        self.logger.warning(f"Unwanted behaviour in mission {self.mission_id} : Arrived at last waypoint during SUSPENDING.")
                    self.state = MissionState.DISCHARGING
                
                case MissionState.DELIVERING:
                    if self.goal_waypoint_idx < len(self.waypoints) - 1:
                        # Waiting before moving to next waypoint
                        self.state = MissionState.WAITING
                    elif self.goal_waypoint_idx == len(self.waypoints) - 1:
                        # Last waypoint reached
                        self.state = MissionState.DISCHARGING
                    else:
                        raise Exception("Invalid waypoint index.")
                
                case _:
                    self.destroy_validation_timer()
                    raise Exception("Arrival event not valid in current state.")
    
    
    def start_validation_timer(self):
        with self.lock:
            if self.timeout_timer is not None:
                self.destroy_validation_timer()
                
            self.timeout_timer = self.node.create_timer(
                self.validation_timeout, 
                self._on_validation_timeout
            )
            
            
    def destroy_validation_timer(self):
        with self.lock:
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
                self.timeout_timer.destroy()
                self.timeout_timer = None
    
    
    def _on_validation_timeout(self):
        with self.lock:
            if self.state in [MissionState.WAITING, MissionState.DISCHARGING]:
                self.logger.warning(f"Mission {self.mission_id} : Validation au waypoint {self.goal_waypoint_idx} automatique après timeout.")
                self.on_validation()
            
            
    # Callback called when validation is received from /human_vaildation topic
    def on_validation(self):
        with self.lock:
            if self.timeout_timer is not None:
                self.destroy_validation_timer()
            
            match self.state:    
                case MissionState.WAITING:
                    self.proceed()
                    
                case MissionState.DISCHARGING:
                    if self.goal_waypoint_idx == len(self.waypoints) - 1:
                        self.complete()
                    elif self.goal_waypoint_idx < len(self.waypoints) - 1:
                        self.release()
                    else:
                        raise Exception("Invalid waypoint index in _on_validation.")
                
                case _:
                    self.logger.warning(f"Unexpected validation for {self.mission_id}.")
            
            
    def on_failure_cb(self):
        with self.lock:
            if self.state not in [MissionState.APPROACHING, MissionState.DELIVERING, MissionState.SUSPENDING]:
                raise TransitionException("Mission can only fail during APPROACHING, DELIVERING or SUSPENDING states.")
            
            self.logger.error(f"Mission {self.mission_id} failed during navigation (previous state: {self.state}).")
            self.state = MissionState.FAILED
                