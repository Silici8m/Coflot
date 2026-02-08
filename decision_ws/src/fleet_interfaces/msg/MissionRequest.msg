# MissionRequest.msg

# DESCRIPTION: Request definition for submitting a new mission to the fleet manager.

std_msgs/Header header

# Unique identifier for the mission (e.g., "mission_123")
string mission_id

# Priority level : higher values indicate higher priority (e.g., 1 for NORMAL, 2 for PRIORITAIRE, 3 for URGENTE)
int32 priority

# Ordered list of target poses the robot must visit
geometry_msgs/Pose[] waypoints