// On instancie les classes vides juste pour vérifier qu'elles existent (optionnel)
const rosManager = new RosManager();
const mapManager = new MapManager();
const uiManager = new UIManager();

// --- VARIABLES GLOBALES ---
var ros;
var viewer;
var gridClient;
var robotMarker;
var waypointCount = 0;

// --- FONCTION DE DEMARRAGE ---
window.initROS = function() {
    // 1. Connexion ROS
    ros = new ROSLIB.Ros({ url : CONFIG.ROSBRIDGE_URL });

    ros.on('connection', function() {
        const ind = document.getElementById('status-indicator');
        ind.innerHTML = "🟢 Connecté";
        ind.style.color = "#00c853";
        ind.style.background = "rgba(0,0,0,0.8)";
        console.log("Connecté à ROS");
    });

    ros.on('error', function(error) {
        document.getElementById('status-indicator').innerHTML = "🔴 Erreur Connexion";
    });

    // 2. Viewer (Carte)
    const mapPanel = document.getElementById('map-panel');
    viewer = new ROS2D.Viewer({
        divID : 'map-panel',
        width : mapPanel.offsetWidth,
        height : mapPanel.offsetHeight,
        background : '#0f0f0f'
    });

    // 3. Client Grid
    gridClient = new ROS2D.OccupancyGridClient({
        ros : ros,
        rootObject : viewer.scene,
        continuous: false
    });

    gridClient.on('change', function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    // 4. Robot Marker
    robotMarker = new createjs.Shape();
    robotMarker.graphics.beginFill("#0088FF").drawPolyStar(0, 0, 0.5, 3, 0, -90);
    robotMarker.visible = false;
    viewer.scene.addChild(robotMarker);

    // 5. Listener Position Robot
    let poseListener = new ROSLIB.Topic({
        ros : ros,
        name : CONFIG.TOPIC_ROBOT,
        messageType : 'geometry_msgs/msg/PoseStamped'
    });

    poseListener.subscribe(function(msg) {
        robotMarker.visible = true;
        robotMarker.x = msg.pose.position.x;
        robotMarker.y = msg.pose.position.y;
        
        // Orientation
        let q = msg.pose.orientation;
        let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        let yaw = Math.atan2(siny_cosp, cosy_cosp);
        robotMarker.rotation = -yaw * (180.0 / Math.PI);

        document.getElementById('robot-info').innerHTML = 
            `X: ${msg.pose.position.x.toFixed(2)} | Y: ${msg.pose.position.y.toFixed(2)}`;
    });

    // Ajout du premier point vide
    addWaypointUI();
};


// --- FONCTIONS UI (Directement copiées de votre version qui marche) ---

function createWaypointTemplate(id) {
    return `<div class="waypoint-card" id="wp-${id}">
        <div class="wp-row-1">
            <select class="wp-select-type" onchange="toggleCoordInput(${id}, this)">
                <option value="custom">Position personnalisée</option>
                <option value="home">Base Démarrage</option>
                <option value="station_a">Station A</option>
                <option value="charge">Zone de Charge</option>
            </select>
            
            <button class="btn-icon btn-save" onclick="savePoint(${id})" title="Enregistrer ce point">
                <img src="save.svg" alt="Save">
            </button>
            
            <button class="btn-icon btn-delete-db" onclick="deleteFromDB(${id})" title="Supprimer de la base">
                <img src="delete.svg" alt="Delete DB">
            </button>

            <button class="btn-icon btn-pick" onclick="console.log('Viser ${id}')" title="Sélectionner sur la carte">
                <img src="cible.svg" alt="Pick Map">
            </button>

            <button class="btn-icon btn-clear" onclick="removeWaypointUI(${id})" title="Enlever de la liste">
                <img src="clear.svg" alt="Clear UI">
            </button>
        </div>

        <div class="wp-row-2">
            <span class="wp-coord-label">X :</span>
            <input type="text" class="wp-coord" name="x" value="0.00">
            <span class="wp-coord-label">Y :</span>
            <input type="text" class="wp-coord" name="y" value="0.00">
        </div>
    </div>`;
}

// Fonction rendue Globale pour le bouton HTML
window.addWaypointUI = function() {
    waypointCount++;
    const container = document.getElementById('waypointsContainer');
    container.insertAdjacentHTML('beforeend', createWaypointTemplate(waypointCount));
    container.scrollTop = container.scrollHeight;
};

window.removeWaypointUI = function(id) { 
    document.getElementById(`wp-${id}`)?.remove(); 
};

window.savePoint = function(id) {
    let name = prompt("Entrez un nom pour enregistrer ce point :");
    if (name) console.log(`Point ${id} enregistré : ${name}`);
};

window.deleteFromDB = function(id) {
    if(confirm("Supprimer de la base ?")) console.log(`Suppression ${id}`);
};

window.toggleCoordInput = function(id, selectEl) { 
    const card = document.getElementById(`wp-${id}`);
    const inputs = card.querySelectorAll('.wp-coord');
    const isCustom = (selectEl.value === 'custom');
    
    inputs.forEach(input => {
        input.disabled = !isCustom;
        input.style.opacity = isCustom ? '1' : '0.5';
    });
};

window.sendMissionToROS = function() { 
    const prio = document.querySelector('input[name="prio"]:checked').value;
    console.log(`Envoi Mission (Priorité: ${prio})`);
    // Code d'envoi ROS à venir
};

// Gestion Resize
window.addEventListener('resize', () => {
    if(viewer && document.getElementById('map-panel')) {
        const panel = document.getElementById('map-panel');
        viewer.resize(panel.offsetWidth, panel.offsetHeight);
    }
});