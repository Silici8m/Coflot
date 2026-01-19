/* ==========================================
   PARTIE 1 : INTERFACE MISSIONS
   ========================================== */

function createWaypointTemplate(id) {
    // J'ai gardé seulement X et Y pour gagner de la place, 
    // dites-moi si vous voulez absolument Z.
    return `<div class="waypoint-card" id="wp-${id}">
        <div class="wp-row-1">
            <select class="wp-select-type" onchange="toggleCoordInput(${id}, this)">
                <option value="custom">Position personnalisée</option>
                <option value="home">Base Démarrage</option>
                <option value="station_a">Station A</option>
                <option value="charge">Zone de Charge</option>
            </select>
            
            <span class="wp-coord-label">x</span>
            <input type="text" class="wp-coord" name="x" value="0">
            
            <span class="wp-coord-label">y</span>
            <input type="text" class="wp-coord" name="y" value="0">
            
            <button class="btn-icon-cible" onclick="console.log('Cible ${id}')" title="Sélectionner sur la carte">
                <img src="cible.svg" alt="Cible">
            </button>
        </div>

        <div class="wp-row-2">
            <div class="wp-check-group">
                <label>Validation</label>
                <input type="checkbox" checked name="validation">
            </div>
            
            <div class="wp-time-group">
                <label>Temps (s)</label>
                <input type="number" class="wp-time-input" value="0" min="0" name="stopTime">
            </div>
            
            <button class="btn-delete" onclick="removeWaypoint(${id})">Supprimer</button>
        </div>
    </div>`;
}

let waypointCount = 0;

function addWaypointUI() {
    waypointCount++;
    const container = document.getElementById('waypointsContainer');
    container.insertAdjacentHTML('beforeend', createWaypointTemplate(waypointCount));
    
    // Scroll vers le bas pour voir le nouveau point
    container.scrollTop = container.scrollHeight;
}

function removeWaypoint(id) { 
    document.getElementById(`wp-${id}`)?.remove(); 
}

window.toggleCoordInput = function(id, selectEl) { 
    // Logique pour griser les champs si "Perso" n'est pas choisi
    const card = document.getElementById(`wp-${id}`);
    const inputs = card.querySelectorAll('.wp-coord');
    const isCustom = (selectEl.value === 'custom');
    
    inputs.forEach(input => {
        input.disabled = !isCustom;
        input.style.opacity = isCustom ? '1' : '0.5';
    });
}

function sendMissionToROS() { 
    console.log("Envoi de la mission à ROS..."); 
    // Ici, vous ajouterez le code pour publier sur un topic ROS
}

// Ajouter un premier point au démarrage
addWaypointUI();


/* ==========================================
   PARTIE 2 : INTEGRATION CARTE ROS
   ========================================== */

let ros;
let viewer;
let gridClient;
let robotMarker;

function initROS() {
    // 1. Connexion
    ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });

    ros.on('connection', () => {
        const ind = document.getElementById('status-indicator');
        ind.innerHTML = "🟢 Connecté à ROS";
        ind.style.color = "#00c853";
        ind.style.background = "rgba(0,0,0,0.8)";
    });

    ros.on('error', () => {
        document.getElementById('status-indicator').innerHTML = "🔴 Erreur Connexion";
    });

    // 2. Viewer
    const mapPanel = document.getElementById('map-panel');
    viewer = new ROS2D.Viewer({
        divID : 'map-panel',
        width : mapPanel.offsetWidth,
        height : mapPanel.offsetHeight,
        background : '#0f0f0f'
    });

    // 3. Grid Client
    gridClient = new ROS2D.OccupancyGridClient({
        ros : ros,
        rootObject : viewer.scene,
        continuous: false
    });

    gridClient.on('change', () => {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    // 4. Robot Marker
    robotMarker = new createjs.Shape();
    robotMarker.graphics.beginFill("#0088FF").drawPolyStar(0, 0, 0.5, 3, 0, -90);
    robotMarker.visible = false;
    viewer.scene.addChild(robotMarker);

    // 5. Listener Position
    let poseListener = new ROSLIB.Topic({
        ros : ros,
        name : '/robot_pose', // Topic simulé
        messageType : 'geometry_msgs/msg/PoseStamped'
    });

    poseListener.subscribe((msg) => {
        robotMarker.visible = true;
        robotMarker.x = msg.pose.position.x;
        robotMarker.y = msg.pose.position.y;
        
        // Orientation
        let q = msg.pose.orientation;
        let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        let yaw = Math.atan2(siny_cosp, cosy_cosp);
        robotMarker.rotation = -yaw * (180.0 / Math.PI);

        // Update UI Info
        document.getElementById('robot-info').innerHTML = 
            `<b>Position:</b><br>X: ${msg.pose.position.x.toFixed(2)}<br>Y: ${msg.pose.position.y.toFixed(2)}`;
    });
}

// Resize automatique
window.addEventListener('resize', () => {
    if(viewer && document.getElementById('map-panel')) {
        const panel = document.getElementById('map-panel');
        viewer.resize(panel.offsetWidth, panel.offsetHeight);
    }
});