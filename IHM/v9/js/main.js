// --- 1. INSTANCIATION ---
const rosManager = new RosManager(); 
const mapManager = new MapManager(); 
const uiManager = new UIManager(); 

// --- 2. VARIABLES GLOBALES ---
var ros;
var waypointCount = 0;

// --- 3. DÉFINITION DES FONCTIONS ---

function createWaypointTemplate(id) {
    // AJOUT DE oninput="updateMapViz()" pour mettre à jour la carte quand on tape
    return `<div class="waypoint-card" id="wp-${id}">
        <div class="wp-row-1">
            <select class="wp-select-type" onchange="toggleCoordInput(${id}, this)">
                <option value="custom">Position personnalisée</option>
                <option value="home">Base Démarrage</option>
                <option value="station_a">Station A</option>
                <option value="charge">Zone de Charge</option>
            </select>
            
            <button class="btn-icon btn-save" onclick="savePoint(${id})" title="Enregistrer">
                <img src="save.svg" alt="S">
            </button>
            <button class="btn-icon btn-delete-db" onclick="deleteFromDB(${id})" title="Supprimer BDD">
                <img src="delete.svg" alt="D">
            </button>
            <button class="btn-icon btn-pick" onclick="startPickMap(${id})" title="Viser">
                <img src="cible.svg" alt="P">
            </button>
            <button class="btn-icon btn-clear" onclick="removeWaypointUI(${id})" title="Enlever">
                <img src="clear.svg" alt="C">
            </button>
        </div>

        <div class="wp-row-2">
            <span class="wp-coord-label">X :</span>
            <input type="text" class="wp-coord" name="x" value="0.00" oninput="updateMapViz()">
            <span class="wp-coord-label">Y :</span>
            <input type="text" class="wp-coord" name="y" value="0.00" oninput="updateMapViz()">
        </div>
    </div>`;
}

// --- NOUVEAU : Fonction qui scanne l'UI et met à jour la carte ---
function updateMapViz() {
    // Si le mapManager n'est pas prêt, on ne fait rien (évite le crash au démarrage)
    if (!mapManager.gridClient || !mapManager.gridClient.currentGrid) return;

    const waypoints = [];
    const wpElements = document.querySelectorAll('.waypoint-card');
    
    wpElements.forEach(card => {
        const x = card.querySelector('input[name="x"]').value;
        const y = card.querySelector('input[name="y"]').value;
        if(!isNaN(parseFloat(x)) && !isNaN(parseFloat(y))) {
            waypoints.push({x: x, y: y});
        }
    });

    // On appelle la fonction qui utilise rosToGrid
    mapManager.displayMissionPath(waypoints);
}

function addWaypointUI() {
    waypointCount++;
    const container = document.getElementById('waypointsContainer');
    if(container) {
        container.insertAdjacentHTML('beforeend', createWaypointTemplate(waypointCount));
        container.scrollTop = container.scrollHeight;
        
        // On essaye de mettre à jour, mais si la map n'est pas là, ça sera ignoré grâce au if dans updateMapViz
        updateMapViz(); 
    }
}

function removeWaypointUI(id) { 
    document.getElementById(`wp-${id}`)?.remove(); 
    updateMapViz(); // Mettre à jour (point supprimé)
}

function savePoint(id) {
    let name = prompt("Nom :");
    if (name) console.log(`Point ${id} saved: ${name}`);
}

function deleteFromDB(id) {
    if(confirm("Supprimer ?")) console.log(`Deleted ${id}`);
}

function toggleCoordInput(id, selectEl) { 
    const card = document.getElementById(`wp-${id}`);
    const inputs = card.querySelectorAll('.wp-coord');
    const isCustom = (selectEl.value === 'custom');
    
    inputs.forEach(input => {
        input.disabled = !isCustom;
        input.style.opacity = isCustom ? '1' : '0.5';
    });
    // On pourrait ajouter des valeurs prédéfinies ici pour "home" etc.
}

function sendMissionToROS() { 
    const prioElement = document.querySelector('input[name="prio"]:checked');
    if(prioElement) console.log(`Envoi Prio: ${prioElement.value}`);
}

function startPickMap(id) {
    console.log(`Visée pour ${id}`);
    mapManager.enablePicking((x, y) => {
        const card = document.getElementById(`wp-${id}`);
        if(card) {
            card.querySelector('input[name="x"]').value = x;
            card.querySelector('input[name="y"]').value = y;
            const sel = card.querySelector('.wp-select-type');
            sel.value = 'custom';
            toggleCoordInput(id, sel);
            
            // IMPORTANT : Mettre à jour le dessin après le choix
            updateMapViz();
        }
    });
}


// --- 4. INIT ---

function initROS() {
    ros = new ROSLIB.Ros({ url : CONFIG.ROSBRIDGE_URL });

    ros.on('connection', function() {
        const ind = document.getElementById('status-indicator');
        if(ind) { ind.innerHTML = "🟢 Connecté"; ind.style.color = "#00c853"; }
        
        mapManager.init(ros, 'map-panel');

        // Simulation Robot (pour voir s'il s'affiche)
        let poseListener = new ROSLIB.Topic({
            ros : ros,
            name : '/robot_pose',
            messageType : 'geometry_msgs/msg/PoseStamped'
        });
        poseListener.subscribe((msg) => {
            mapManager.updateRobot(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation);
            document.getElementById('robot-info').innerHTML = `X: ${msg.pose.position.x.toFixed(2)}`;
        });
    });

    ros.on('error', function() {
        const ind = document.getElementById('status-indicator');
        if(ind) ind.innerHTML = "🔴 Erreur";
    });

    addWaypointUI();
}

// --- 5. EXPOSITION ---
window.initROS = initROS;
window.addWaypointUI = addWaypointUI;
window.removeWaypointUI = removeWaypointUI;
window.savePoint = savePoint;
window.deleteFromDB = deleteFromDB;
window.toggleCoordInput = toggleCoordInput;
window.sendMissionToROS = sendMissionToROS;
window.startPickMap = startPickMap;
window.updateMapViz = updateMapViz; // Ne pas oublier celle-ci

window.addEventListener('resize', () => { mapManager.resize(); });