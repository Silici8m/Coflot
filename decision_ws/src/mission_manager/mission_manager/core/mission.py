# mission.py

from enum import Enum
import threading
from typing import List, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.timer import Timer

from mission_manager.config import VALIDATION_TIMEOUT

class MissionState(Enum):
    """
    Enumeration representing the possible states of a mission lifecycle.

    Attributes:
        PENDING: Mission is created but not yet active or has been revoked/released.
        ASSIGNED: Mission is assigned to a robot but hasn't started moving.
        APPROACHING: Robot is moving towards the first waypoint (pickup point).
        WAITING: Robot arrived at a waypoint and waits for validation to proceed.
        DELIVERING: Robot is moving between intermediate waypoints.
        SUSPENDING: Mission is being paused; robot is moving to a safe halt or waypoint.
        DISCHARGING: Robot arrived at the final destination or is unloading.
        FINISHED: Mission completed successfully.
        FAILED: Mission failed due to navigation or operational errors.
    """
    PENDING = 0
    ASSIGNED = 1
    APPROACHING = 2
    WAITING = 3
    DELIVERING = 4
    SUSPENDING = 5
    DISCHARGING = 6
    FINISHED = 7
    FAILED = 8
    
class MissionPriority(Enum):
    """
    Enumeration representing the priority levels of a mission.

    Attributes:
        NORMAL: Standard priority.
        PRIORITAIRE: High priority.
        URGENTE: Critical priority, may preempt other missions.
    """
    NORMAL = 1
    PRIORITAIRE = 2
    URGENTE = 3

class TransitionException(Exception):
    """
    Exception raised when an invalid state transition is attempted.
    """
    pass
    
    
class Mission:
    """
    Manages the lifecycle, state, and execution of a specific mission.

    This class encapsulates all data related to a mission (waypoints, priority, state)
    and manages the interactions with the assigned robot via a robot adapter. It handles
    navigation events, timeouts, and human validation logic.

    Attributes:
        node (Node): Reference to the parent ROS 2 node for logging and timer creation.
        logger (rclpy.impl.rcutils_logger.RcutilsLogger): Logger instance.
        mission_id (str): Unique identifier for the mission.
        priority (int): Priority level of the mission.
        state (MissionState): Current state of the mission.
        waypoints (list): List of geometry_msgs/Pose representing the path.
        assigned_robot_id (Optional[str]): ID of the robot currently executing the mission.
        goal_waypoint_idx (Optional[int]): Index of the current target waypoint.
        request_stamp (Time): Timestamp when the mission was requested.
        robot_adapter (Optional[Any]): Interface to control the assigned robot.
        timeout_timer (Optional[Timer]): Timer for auto-validation at waypoints.
        validation_timeout (float): Duration in seconds before auto-validation.
        lock (threading.RLock): Reentrant lock for thread-safe state management.
    """
    node: Node
    mission_id: str
    priority: int
    state: MissionState
    waypoints: list
    assigned_robot_id: Optional[str]
    goal_waypoint_idx: Optional[int]
    request_stamp: Time
    #client: RobotNavigationActionClient
    timeout_timer: Optional[Timer]

    def __init__(self, parent_node: Node, mission_id: str, priority: int, waypoints: list, request_stamp: Time) -> None:
        """
        Initializes a new Mission instance.

        Args:
            parent_node (Node): The ROS 2 node managing this mission.
            mission_id (str): Unique identifier.
            priority (int): Priority value (usually mapped from MissionPriority).
            waypoints (list): List of target poses.
            request_stamp (Time): Time of creation.
        """
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
        self.validation_timeout = VALIDATION_TIMEOUT
        self.lock = threading.RLock()
        
        
    def assign(self, robot_id: str, robot_adapter: Any) -> None:
        """
        Assigns a specific robot to this mission.

        Args:
            robot_id (str): The identifier of the robot.
            robot_adapter (Any): The adapter instance to control the robot.

        Raises:
            TransitionException: If the mission is not in PENDING state.
            Exception: If the robot adapter indicates the robot is busy.
        """
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
        
        
    def start_approaching(self) -> None:
        """
        Commands the robot to move towards the first waypoint (pickup location).

        Transitions the mission state from ASSIGNED to APPROACHING.

        Raises:
            TransitionException: If the mission is not in ASSIGNED state.
            Exception: If the robot is busy executing another action.
        """
        with self.lock:
            if self.state != MissionState.ASSIGNED:
                raise TransitionException("Mission can only start approaching from ASSIGNED state.")
            
            if self.robot_adapter.is_busy():
                self.logger.warning(f"Robot {self.assigned_robot_id} is busy, cannot start mission {self.mission_id}.")
                raise Exception("Cannot start mission with a robot whose action server is busy.")
            
            self.robot_adapter.send_goal(
                self.waypoints[self.goal_waypoint_idx],
                on_success=self.on_arrival_cb,
                on_failure=self.on_failure_cb
            )
            self.state = MissionState.APPROACHING
            
            
    def revoke(self) -> None:
        """
        Cancels the mission while it is in the approaching phase.

        Stops the robot and resets the mission state to PENDING, allowing reassignment.

        Raises:
            TransitionException: If the mission is not in APPROACHING state.
        """
        with self.lock:
            if self.state !=  MissionState.APPROACHING:
                raise TransitionException("Mission can only be revoked from APPROACHING state.")
            
            self._release_robot(cancel_active_goal=True)
            self.state = MissionState.PENDING


    def proceed(self) -> None:
        """
        Commands the robot to proceed to the next waypoint in the list.

        Transitions the mission state from WAITING to DELIVERING.

        Raises:
            TransitionException: If the mission is not in WAITING state.
            Exception: If the next waypoint index is out of bounds.
        """
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
    
    
    def suspend(self) -> None:
        """
        Requests the mission to pause execution.

        Transitions state to SUSPENDING. If the robot is already waiting, it simulates
        an arrival to trigger the suspending logic immediately.

        Note:
            Suspension is ignored if the robot is currently traveling to the final waypoint.

        Raises:
            TransitionException: If not in DELIVERING or WAITING state.
        """
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


    def release(self) -> None:
        """
        Releases the robot from the current mission, making the mission pending again.

        Used when a mission is interrupted but can be resumed later. Transitions
        state from DISCHARGING to PENDING.

        Raises:
            TransitionException: If the mission is not in DISCHARGING state.
        """
        with self.lock:
            if self.state != MissionState.DISCHARGING:
                raise TransitionException("Mission can only be released from DISCHARGING state.")
            
            self._release_robot(cancel_active_goal=False)
            self.state = MissionState.PENDING
            self.logger.info(f"Mission {self.mission_id} released. Resumable at waypoint {self.goal_waypoint_idx}.")
            
            
    def complete(self) -> None:
        """
        Marks the mission as successfully finished and releases the robot.

        Transitions state from DISCHARGING to FINISHED.

        Raises:
            TransitionException: If the mission is not in DISCHARGING state.
        """
        with self.lock:
            if self.state != MissionState.DISCHARGING:
                raise TransitionException("Mission can only be completed from DISCHARGING state.")
            
            self._release_robot(cancel_active_goal=False)
            self.state = MissionState.FINISHED
            self.logger.info(f"Mission {self.mission_id} finished successfully.")

    
    def _release_robot(self, cancel_active_goal: bool) -> None:
        """
        Internal method to decouple the robot from the mission.

        Args:
            cancel_active_goal (bool): If True, sends a cancellation request for the
                                       current navigation goal.
        """
        with self.lock:
            if self.robot_adapter is not None:
                if cancel_active_goal and self.robot_adapter.is_navigating():
                    self.robot_adapter.cancel_goal()
                    
                self.assigned_robot_id = None
                self.robot_adapter = None


    def on_arrival_cb(self) -> None:
        """
        Callback executed when the robot successfully reaches a target waypoint.

        Handles state transitions based on the current mission phase (APPROACHING,
        DELIVERING, SUSPENDING) and starts the validation timer.

        Raises:
            TransitionException: If called in an invalid state.
            Exception: If an invalid waypoint index is encountered.
        """
        if self.state == MissionState.PENDING:
            # Cas d'un revoke
            return
        with self.lock:
            if self.state not in [MissionState.APPROACHING, MissionState.DELIVERING, MissionState.SUSPENDING]:
                raise TransitionException(f"Arrival event not valid in current state. {self.state.name}")
            
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
    
    
    def start_validation_timer(self) -> None:
        """
        Starts or restarts the timer for automatic validation at a waypoint.
        """
        with self.lock:
            if self.timeout_timer is not None:
                self.destroy_validation_timer()
                
            self.timeout_timer = self.node.create_timer(
                self.validation_timeout, 
                self._on_validation_timeout
            )
            
            
    def destroy_validation_timer(self) -> None:
        """
        Cancels and destroys the active validation timer if it exists.
        """
        with self.lock:
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
                self.timeout_timer.destroy()
                self.timeout_timer = None
    
    
    def _on_validation_timeout(self) -> None:
        """
        Internal callback triggered when the validation timer expires.

        Automatically triggers validation if the mission is waiting or discharging.
        """
        with self.lock:
            if self.state in [MissionState.WAITING, MissionState.DISCHARGING]:
                self.logger.warning(f"Mission {self.mission_id} : Validation au waypoint {self.goal_waypoint_idx} automatique après timeout.")
                self.on_validation()
            
            
    # Callback called when validation is received from /human_vaildation topic
    def on_validation(self) -> None:
        """
        Processes a validation signal (manual or timeout).

        Depending on the current state, this method advances the mission (proceed),
        completes it, or releases the robot.

        Raises:
            Exception: If the waypoint index is invalid during DISCHARGING.
        """
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
            
            
    def on_failure_cb(self) -> None:
        """
        Callback executed when the robot fails to reach a goal.

        Transitions the mission state to FAILED.

        Raises:
            TransitionException: If the failure occurs in a state where navigation is not active.
        """
        with self.lock:
            if self.state == MissionState.PENDING:
                # Cas d'un revoke
                return
            if self.state not in [MissionState.APPROACHING, MissionState.DELIVERING, MissionState.SUSPENDING]:
                raise TransitionException("Mission can only fail during APPROACHING, DELIVERING or SUSPENDING states.")

            self.logger.error(f"Mission {self.mission_id} failed during navigation (previous state: {self.state}).")
            self.state = MissionState.FAILED