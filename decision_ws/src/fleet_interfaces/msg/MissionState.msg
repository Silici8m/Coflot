# MissionState.msg

string mission_id

# Priorité de la mission 0 = Normal, 1 = Prioritaire, 2 = Urgente
int32 priority

# Liste des points de passage de la mission
geometry_msgs/Pose[] waypoints

# Robot réalisant la mission ("" si non assigné)
string assigned_robot_id

# États possibles :     PENDING = 0, ASSIGNED = 1, APPROACHING = 2, WAITING = 3, DELIVERING = 4, SUSPENDING = 5, DISCHARGING = 6, FINISHED = 7, FAILED = 8
string status 

# Index du waypoint objectif (-1 si la mission n'a pas encore commencé, 0 avant le premier waypoint, etc. L'index est incrémenté lorsque le robot quitte le waypoint)
int32 goal_waypoint_idx

# Stamp de création de la mission
builtin_interfaces/Time request_stamp