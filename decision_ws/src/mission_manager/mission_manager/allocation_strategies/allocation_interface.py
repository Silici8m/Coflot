# allocation_interface.py

from enum import Enum
from dataclasses import dataclass

class AllocationAction(Enum):
    """
    Defines the specific type of action to be performed by the Dispatcher.

    This enumeration allows the Dispatcher to apply the correct procedure
    (start, preemption, suspension) without inferring logic.

    Attributes:
        ASSIGN_AND_START: Standard assignment. The robot is free and starts the mission immediately.
        REVOKE: Preemption (Emergency). The current mission is cancelled to start the new one.
        SUSPEND: Suspension. The current mission is paused to start the new one.
        NOTHING: No action required.
    """
    ASSIGN_AND_START = 1
    REVOKE = 2
    SUSPEND = 3
    NOTHING = 4

@dataclass
class AllocationDecision:
    """
    Data Transfer Object (DTO) representing a decision made by the allocator.

    Attributes:
        action (AllocationAction): The type of action to execute.
        mission_id (str): The unique identifier of the mission concerned by the decision.
        robot_id (str): The unique identifier of the robot assigned or affected.
        reason (str): A textual explanation for the decision (default is empty).
    """
    action: AllocationAction
    mission_id: str
    robot_id: str
    reason: str = ""