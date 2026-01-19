// --- 1. INSTANCIATION DES MANAGERS ---
const rosManager = new RosManager(); 
const mapManager = new MapManager(); 
const uiManager = new UIManager(); 

// --- 2. VARIABLES GLOBALES ---
var ros;
var waypointCount = 0;

// --- 3. DÉFINITION DES FONCTIONS (Logique) ---

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

            <button class="btn-icon btn-pick" onclick="startPickMap(${id})" title="Sélectionner sur la carte">
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

function addWaypointUI() {
    waypointCount++;
    const container = document.getElementById('waypointsContainer');
    if(container) {
        container.insertAdjacentHTML('beforeend', createWaypointTemplate(waypointCount));
        container.scrollTop = container.scrollHeight;
    }
}

function removeWaypointUI(id) { 
    document.getElementById(`wp-${id}`)?.remove(); 
}

function savePoint(id) {
    let name = prompt("Entrez un nom pour enregistrer ce point :");
    if (name) console.log(`Point ${id} enregistré : ${name}`);
}

function deleteFromDB(id) {
    if(confirm("Supprimer de la base ?")) console.log(`Suppression ${id}`);
}

function toggleCoordInput(id, selectEl) { 
    const card = document.getElementById(`wp-${id}`);
    const inputs = card.querySelectorAll('.wp-coord');
    const isCustom = (selectEl.value === 'custom');
    
    inputs.forEach(input => {
        input.disabled = !isCustom;
        input.style.opacity = isCustom ? '1' : '0.5';
    });
}

function sendMissionToROS() { 
    const prioElement = document.querySelector('input[name="prio"]:checked');
    if(prioElement) {
        console.log(`Envoi Mission (Priorité: ${prioElement.value})`);
    }
}

// --- FONCTION DE VISÉE (Lien avec MapManager) ---
function startPickMap(id) {
    console.log(`Demande de visée pour le point ${id}`);
    
    // On active le mode picking via le manager
    mapManager.enablePicking((x, y) => {
        const card = document.getElementById(`wp-${id}`);
        if(card) {
            card.querySelector('input[name="x"]').value = x;
            card.querySelector('input[name="y"]').value = y;
            
            const sel = card.querySelector('.wp-select-type');
            sel.value = 'custom';
            toggleCoordInput(id, sel);
        }
    });
}


// --- 4. FONCTION PRINCIPALE (Init) ---

function initROS() {
    // Connexion
    ros = new ROSLIB.Ros({ url : CONFIG.ROSBRIDGE_URL });

    ros.on('connection', function() {
        const ind = document.getElementById('status-indicator');
        if(ind) {
            ind.innerHTML = "🟢 Connecté";
            ind.style.color = "#00c853";
            ind.style.background = "rgba(0,0,0,0.8)";
        }
        
        // Initialisation du MapManager (qui gère la carte et le picking)
        mapManager.init(ros, 'map-panel');
    });

    ros.on('error', function(error) {
        const ind = document.getElementById('status-indicator');
        if(ind) ind.innerHTML = "🔴 Erreur Connexion";
    });

    // On ajoute un point vide au démarrage
    addWaypointUI();
}


// --- 5. EXPOSITION AU HTML (Window) ---
// C'est ici qu'on rend les fonctions accessibles aux boutons HTML (onclick="")

window.initROS = initROS;
window.addWaypointUI = addWaypointUI;
window.removeWaypointUI = removeWaypointUI;
window.savePoint = savePoint;
window.deleteFromDB = deleteFromDB;
window.toggleCoordInput = toggleCoordInput;
window.sendMissionToROS = sendMissionToROS;
window.startPickMap = startPickMap;

// Gestion du redimensionnement
window.addEventListener('resize', () => {
    mapManager.resize();
});