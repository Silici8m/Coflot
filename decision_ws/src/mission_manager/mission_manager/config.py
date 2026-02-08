"""
Configuration parameters for the Mission Manager system.

This module defines global constants used across the decision workspace, including:
- Physical parameters for cost estimation (speed, waiting times).
- Coefficients for the Utility allocation strategy.
- Timeouts for ROS 2 actions and timers.
- ROS 2 Topic names and Frame IDs.
"""

# --- Robot Model & Physical Parameters ---

# Average speed of the robot in meters per second. 
# Used to estimate travel time in cost calculations.
ROBOT_AVERAGE_SPEED: float = 0.3

# Estimated average time in seconds for static tasks (e.g., loading, unloading, 
# or transition delays). Added to the cost when a stop is required.
AVERAGE_WAITING_TIME: float = 3.0

# Base "Quality" score in seconds used for Utility calculation.
# Utility = Quality - Cost. Determines the baseline score before penalties.
ROBOT_QUALITY_S: float = 100.0

# Penalty cost (in seconds) applied when reassigning (stealing) a mission 
# that is already in the 'APPROACHING' state.
STEAL_COST: float = 1.0


# --- Allocation Strategy Coefficients ---

# Multiplier applied to the utility score for missions with 'PRIORITAIRE' priority.
COEF_MISSION_PRIORITAIRE: float = 1.5

# Multiplier applied to the utility score for missions with 'URGENTE' priority.
COEF_MISSION_URGENTE: float = 5.0


# --- ROS 2 Frames ---

# The reference frame ID used for all PoseStamped messages (usually 'map').
POSESTAMPED_FRAME_ID: str = 'map'


# --- Timeouts & Durations ---

# Duration in seconds before a mission in 'WAITING' state is automatically validated.
VALIDATION_TIMEOUT: float = 30.0

# Timeout in seconds to wait for the Action Server to accept a path computation request.
REQUEST_PATH_COMPUTING_TIMEOUT: float = 0.1

# Timeout in seconds to wait for the Action Server to return the computed path result.
COMPUTE_PLAN_TIMEOUT: float = 0.5

# Interval in seconds for the timer responsible for removing FINISHED/FAILED missions from the registry.
CLEAR_FINISHED_PERIOD: int = 5


# --- Execution Intervals ---

# Frequency in seconds of the main allocation loop (decision making).
ALLOCATION_INTERVAL: float = 1.0

# Frequency in seconds for publishing the global mission state to ROS topics.
MISSION_PUBLICATION_INTERVAL: float = 0.2


# --- ROS 2 Topic Names ---

# Topic for receiving telemetry updates from the fleet (fleet_interfaces/RobotStateArray).
FLEET_STATE_TOPIC: str = '/fleet/fleet_state'

# Topic for receiving new mission submissions (fleet_interfaces/MissionRequest).
MISSION_REQUEST_TOPIC: str = '/mission/mission_request'

# Topic for receiving manual validation signals (std_msgs/String).
VALIDATION_TOPIC: str = '/mission/validation'

# Topic for publishing the aggregated state of all missions (fleet_interfaces/MissionStateArray).
MISSIONS_STATE_TOPIC: str = '/mission/missions_state'