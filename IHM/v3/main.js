// --- CONFIGURATION ---
const ROSBRIDGE_URL = 'ws://localhost:9090';
const MAP_TOPIC = '/map';
const ODOM_TOPIC = '/odom'; // Vérifiez si c'est /odom ou /amcl_pose

// 1. Connexion à ROS
var ros = new ROSLIB.Ros({
    url : ROSBRIDGE_URL
});

ros.on('connection', function() {
    var statusElem = document.getElementById("status");
    statusElem.innerHTML = "Connecté à ROS 2 (Port 9090)";
    statusElem.style.color = "green";
});

ros.on('error', function(error) {
    document.getElementById("status").innerHTML = "Erreur Websocket (Lancer rosbridge_server)";
});

ros.on('close', function() {
    document.getElementById("status").innerHTML = "Connexion fermée";
});

// 2. Création du Viewer (La zone de dessin)
var viewer = new ROS2D.Viewer({
    divID : 'map',
    width : 800,
    height : 600,
    background: '#7f7f7f' // Gris standard pour l'inconnu
});

// 3. Gestion de la Map
var gridClient = new ROS2D.OccupancyGridClient({
    ros : ros,
    rootObject : viewer.scene,
    continuous: false // Mettre à true si c'est du SLAM en temps réel
});

// Centrer la vue quand la map arrive
gridClient.on('change', function() {
    viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
});

// 4. Gestion du Robot (Triangle Bleu)
// On dessine un triangle simple
var robotMarker = new createjs.Shape();
robotMarker.graphics.beginFill("blue").drawPolyStar(0, 0, 0.5, 3, 0, -90); // Taille 0.5m
robotMarker.visible = false; // Caché tant qu'on a pas de position
viewer.scene.addChild(robotMarker);

// 5. Abonnement à la position (Odometry)
var odomListener = new ROSLIB.Topic({
    ros : ros,
    name : ODOM_TOPIC,
    messageType : 'nav_msgs/msg/Odometry'
});

odomListener.subscribe(function(message) {
    // Rendre le robot visible dès qu'on reçoit une donnée
    robotMarker.visible = true;

    // Position (X, Y)
    robotMarker.x = message.pose.pose.position.x;
    robotMarker.y = message.pose.pose.position.y;

    // Orientation (Quaternion -> Euler/Degrés)
    var q = message.pose.pose.orientation;
    // Formule mathématique pour obtenir l'angle de lacet (Yaw)
    var siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    var cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    var yaw = Math.atan2(siny_cosp, cosy_cosp);

    // Conversion Radian -> Degrés et inversion pour l'affichage Canvas
    robotMarker.rotation = -yaw * (180.0 / Math.PI);
});