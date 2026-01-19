class RosManager {
    constructor() {
        this.ros = null;
        this.fleetStateTopic = null;
        this.missionRequestTopic = null;

        // Variables pour le throttling (limitation de FPS)
        this.lastFleetUpdate = 0;
        this.fleetUpdateInterval = 200; // ms (100ms = 10 FPS max)
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
        console.log(`Abonnement au topic ${CONFIG.TOPIC_FLEET} (Type: ${CONFIG.TYPE_FLEET})...`);

        this.fleetStateTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_FLEET,
            messageType: CONFIG.TYPE_FLEET 
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

    // Envoie la mission
    advertiseMissionRequest() {
        if (!this.ros) {
            console.error("Tentative d'abonnement sans connexion ROS active.");
            return;
        }
        console.log(`Abonnement au topic ${CONFIG.TOPIC_MISSION} (Type: ${CONFIG.TYPE_MISSION})...`);

        this.missionRequestTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_MISSION,
            messageType: CONFIG.TYPE_MISSION
        });

        this.missionRequestTopic.advertise();
    }

    
    publishMission(missionData) {
        if (!this.missionRequestTopic || !this.missionRequestTopic.isAdvertised) {
            console.error("Tentative de publication sans abonnement au topic de mission.");
            return;
        }
        const message = new ROSLIB.Message(missionData);
        this.missionRequestTopic.publish(message);
        console.log(`Mission ${missionData.name} publiée sur ${CONFIG.TOPIC_MISSION}.`);
        console.log(message);
    }
}