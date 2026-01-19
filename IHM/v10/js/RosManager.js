class RosManager {
    constructor() {
        this.ros = null;
        this.robotListener = null;
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

    // S'abonne à la position du robot et renvoie les données via un callback
    subscribeRobotPose(callback) {
        if (!this.ros) return;
        
        this.robotListener = new ROSLIB.Topic({
            ros: this.ros,
            name: CONFIG.TOPIC_ROBOT,
            messageType: 'geometry_msgs/msg/PoseStamped'
        });

        this.robotListener.subscribe((msg) => {
            // On extrait juste les données utiles
            const x = msg.pose.position.x;
            const y = msg.pose.position.y;
            const orientation = msg.pose.orientation;
            callback(x, y, orientation);
        });
    }

    // Envoie la mission (Logique future)
    sendMission(priority, waypoints) {
        console.log(`[ROS] Envoi Mission - Prio: ${priority}, Points: ${waypoints.length}`);
        // Ici viendra le code ROSLIB.Topic.publish(...)
    }
}