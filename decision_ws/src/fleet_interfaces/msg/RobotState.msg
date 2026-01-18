# RobotState.msg
# État physique d'un robot

string robot_id
string mode # Non utilisé pour l'instant
float32 battery_pct  # Pourcentage batterie
geometry_msgs/Pose pose
builtin_interfaces/Time stamp # Type à changé eventuellement