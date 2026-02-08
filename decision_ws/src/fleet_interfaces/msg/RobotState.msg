# RobotState.msg

# DESCRIPTION: Telemetry and status data for a single robot.

# Unique identifier of the robot (namespace)
string robot_id

# Current operational mode (Not used)
string mode

# Battery level percentage (0.0 to 100.0)
float32 battery_pct

# Current map-referenced pose
geometry_msgs/Pose pose

# Timestamp of the data
builtin_interfaces/Time stamp