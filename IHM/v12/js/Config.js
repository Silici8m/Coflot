const CONFIG = {
    ROSBRIDGE_URL: 'ws://localhost:9090',
    TYPE_POSE:     'geometry_msgs/msg/PoseStamped',

    TOPIC_MISSION_REQUEST: '/mission/mission_request',
    TYPE_MISSION_REQUEST: 'fleet_interfaces/msg/MissionRequest',

    TOPIC_FLEET_STATE:   '/fleet/fleet_state',
    TYPE_FLEET_STATE:   'fleet_interfaces/msg/RobotStateArray' ,

    TOPIC_MISSIONS:   '/mission/missions_state',
    TYPE_MISSIONS:   'fleet_interfaces/msg/RobotStateArray' 
};