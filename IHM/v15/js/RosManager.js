// RosManager.js

class RosManager {
    constructor() {
        this.ros = null;
        this.fleetStateTopic = null;
        this.missionRequestTopic = null;
        this.missionsStateTopic = null;

        // Variables pour le throttling (limitation de FPS)
        this.lastFleetUpdate = 0;
        this.fleetUpdateInterval = 200; // ms (100ms = 10 FPS max)

        this.lastMissionsUpdate = 0;
        this.missionsUpdateInterval = 500; // ms
    }

    // Connecte à ROS et gère les callbacks succès/erreur
    connect(onConnected, onError) {
        this.ros = new ROSLIB.Ros({ url: CONFIG.ROSBRIDGE_URL });

        this.ros.on('connection', () => {
            console.log("RosManager: Connecté");
            if (onConnected) onConnected();
        });

        this.ros.on('error', (error) => {
            console.error("RosManager: Erreur", error);
            if (onError) onError(error);
        });
        
        this.ros.on('close', () => {
            console.log("RosManager: Connexion fermée");
        });
    }

    // Abonnement à la flotte avec Throttling
    subscribeFleet(callback) {
        if (!this.ros) {
            console.error("Tentative d'abonnement sans connexion ROS active.");
            return;
        }
        console.log(`Abonnement au topic ${CONFIG.TOPIC_FLEET_STATE} (Type: ${CONFIG.TYPE_FLEET_STATE})...`);

        this.fleetStateTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_FLEET_STATE,
            messageType: CONFIG.TYPE_FLEET_STATE 
        });
        this.fleetStateTopic.subscribe((msg) => {
            const now = Date.now();
            // On ne déclenche le callback que si le temps écoulé > intervalle
            if (now - this.lastFleetUpdate > this.fleetUpdateInterval) {
                callback(msg);
                this.lastFleetUpdate = now;
            }
        });
    }

    subscribeMissions(callback) {
        if (!this.ros) return;

        console.log(`Abonnement au topic ${CONFIG.TOPIC_MISSIONS_STATE} (Type: ${CONFIG.TYPE_MISSIONS_STATE})...`);

        this.missionsStateTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_MISSIONS_STATE,
            messageType: CONFIG.TYPE_MISSIONS_STATE 
        });

        this.missionsStateTopic.subscribe((msg) => {
            const now = Date.now();
            if (now - this.lastMissionsUpdate > this.missionsUpdateInterval) {
                callback(msg);
                this.lastMissionsUpdate = now;
            }
        });
    }


    // Envoie la mission
    advertiseMissionRequest() {
        if (!this.ros) {
            console.error("Tentative d'abonnement sans connexion ROS active.");
            return;
        }
        console.log(`Abonnement au topic ${CONFIG.TOPIC_MISSION_REQUEST} (Type: ${CONFIG.TYPE_MISSION_REQUEST})...`);

        this.missionRequestTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_MISSION_REQUEST,
            messageType: CONFIG.TYPE_MISSION_REQUEST
        });

        this.missionRequestTopic.advertise();
    }

    
    publishMissionRequest(missionData) {
        if (!this.missionRequestTopic || !this.missionRequestTopic.isAdvertised) {
            console.error("Tentative de publication sans abonnement au topic de mission.");
            return;
        }
        const now = new Date();
        console.log(CONFIG.MISSION_REQUEST_FRAME_ID);
        const header = {
            stamp: {
                secs: Math.floor(now.getTime() / 1000),
                nsecs: (now.getTime() % 1000) * 1000000
            },
            frame_id: CONFIG.MISSION_REQUEST_FRAME_ID
        };
        missionData.header = header
        const message = new ROSLIB.Message(missionData);
        this.missionRequestTopic.publish(message);
        console.log(`Mission ${missionData.name} publiée sur ${CONFIG.TOPIC_MISSION}.`);
        console.log(message);
    }
    advertiseMissionValidation() {
        if (!this.ros) return;
        
        this.missionValidationTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_MISSION_VALIDATION,
            messageType: CONFIG.TYPE_MISSION_VALIDATION
        });
        
        this.missionValidationTopic.advertise();
    }

    sendValidation(missionId) {
        if (!this.missionValidationTopic) {
            console.error("Le topic de validation n'est pas initialisé.");
            return;
        }

        const msg = new ROSLIB.Message({
            data: missionId
        });

        this.missionValidationTopic.publish(msg);
        console.log(`Validation envoyée pour la mission : ${missionId}`);
    }
}