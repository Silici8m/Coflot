const CONFIG = {
    ROSBRIDGE_URL: 'ws://localhost:9090',
    TYPE_POSE:     'geometry_msgs/msg/PoseStamped',

    TOPIC_MISSION: '/fleet/mission_request',
    TYPE_MISSION: 'fleet_simulator/msg/MissionRequest',

    TOPIC_FLEET:   '/fleet/fleet_state',
    TYPE_FLEET:   'fleet_simulator/msg/FleetState' 
};