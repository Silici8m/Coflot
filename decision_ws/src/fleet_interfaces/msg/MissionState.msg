string mission_id

# Priorité de la mission 0 = Normal, 1 = Prioritaire, 2 = Urgente
int32 priority

# Liste des points de passage de la mission
geometry_msgs/Pose[] waypoints

# Robot réalisant la mission ("" si non assigné)
string assigned_robot_id

# États possibles : PENDING, EXECUTING, WAITING_VALIDATION, COMPLETED, TRUNCATED, FAILED
string status 

# Index du waypoint courant (-1 si la mission n'a pas encore commencé, 0 avant le premier waypoint, etc.)
int32 current_waypoint_idx

# Si True : Le robot doit s'arrêter au prochain waypoint pour une interruption contrôlée.
bool interruption_requested

# Stamp de création de la mission
builtin_interfaces/Time request_stamp