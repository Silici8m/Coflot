# MissionState.msg

# DESCRIPTION: Live status of a specific mission managed by the system.

string mission_id

# Priority level: 0 = Normal, 1 = High, 2 = Urgent
int32 priority

# Sequence of waypoints to visit
geometry_msgs/Pose[] waypoints

# ID of the robot performing the mission (empty string "" if unassigned)
string assigned_robot_id

# Lifecycle status.
# Values: PENDING, ASSIGNED, APPROACHING, WAITING, DELIVERING, 
#         SUSPENDING, DISCHARGING, FINISHED, FAILED
string status 

# Current target waypoint index.
# -1: Not started / 0: Moving to 1st waypoint / Increments upon departure.
int32 goal_waypoint_idx

# Time when the mission was originally requested
builtin_interfaces/Time request_stamp