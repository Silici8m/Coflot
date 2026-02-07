// main.js

// --- 1. INSTANCIATION ---
const rosManager = new RosManager();
const mapManager = new MapManager();
const uiManager  = new UIManager();
const missionManager = new MissionManager();

// --- 2. INIT GLOBAL (Appelé par body onload) ---
function initProject() {
    // A. Connexion
    rosManager.connect(
        () => { // Succès
            uiManager.updateStatus(true);
            
            // On lance la map une fois connecté
            mapManager.init(rosManager.ros, 'map-panel');

            // On s'abonne au robot
            rosManager.subscribeFleet((fleetMsg) => {
                // 1. Mise à jour graphique
                mapManager.updateFleet(fleetMsg);
                
                // 2. Mise à jour info texte (Panel de droite)
                // On peut faire une fonction simple dans UIManager pour lister les robots
                let infoHtml = "";
                fleetMsg.robots.forEach(r => {
                    infoHtml += `<b>${r.robot_id}</b>: ${r.mode} (${Math.round(r.battery_pct)}%)<br>`;
                });
                document.getElementById('robot-info').innerHTML = infoHtml;
            });

            rosManager.subscribeMissions((missionArrayMsg) => {
                
                missionManager.update(missionArrayMsg.missions);
                mapManager.updateMissions(missionArrayMsg.missions);
            });

            rosManager.advertiseMissionRequest();
            rosManager.advertiseMissionValidation();
        },
        () => { // Erreur
            uiManager.updateStatus(false);
            mapManager.hideRobot(); 
        }
    );



    // B. Ajout du premier point
    uiManager.addWaypoint();
}

// --- 3. FONCTIONS GLOBALES (Appelées par le HTML) ---
// Ces fonctions servent de "pont" entre le clic HTML et les Managers

window.updateMapViz = function() {
    // 1. UI Manager donne les données
    const points = uiManager.getAllWaypoints();
    // 2. Map Manager affiche
    mapManager.displayMissionPath(points);
};

window.addWaypointUI = function() {
    uiManager.addWaypoint();
    window.updateMapViz();
};

window.removeWaypointUI = function(id) {
    uiManager.removeWaypoint(id);
    window.updateMapViz();
};

window.savePoint = function(id) {
    // À connecter plus tard au RosManager (Services)
    console.log("Save", id);
};

window.deleteFromDB = function(id) {
    console.log("Delete DB", id);
};

window.validateMission = function(missionId) {
    rosManager.sendValidation(missionId);
};

window.startPickMap = function(id) {
    // 1. On active le picking sur la Map
    mapManager.enablePicking((x, y) => {
        // 2. Quand on a cliqué, on met à jour l'UI
        uiManager.setWaypointCoords(id, x, y);
        // 3. Et on met à jour le dessin
        window.updateMapViz();
    });
};

window.sendMissionToROS = function() {
    const name = uiManager.getName() ? uiManager.getName() : "mission_" + Date.now();
    
    // IMPORTANT : Conversion en Entier pour int32
    const prioRaw = uiManager.getPriority(); 
    const prio = parseInt(prioRaw, 10); 

    const points = uiManager.getAllWaypoints();
    
    // Vérification console avant envoi
    console.log("Envoi payload:", { mission_id: name, priority: prio, waypoints: points });

    rosManager.publishMissionRequest({ 
        mission_id: name, 
        priority: prio, 
        waypoints: points 
    });

    // uiManager.clearMission(); // Commentez temporairement pour debugger si besoin
    window.updateMapViz();
};

function updateMissionsUI(missions) {
    const container = document.getElementById('missions-list'); // Assure-toi que cet ID existe dans ton HTML
    if (!container) return;

    let html = "<h3>Suivi des Missions</h3>";
    missions.forEach(m => {
        const color = m.assigned_robot_id ? "#d4edda" : "#fff3cd"; // Vert si assigné, jaune si en attente
        html += `
            <div style="border: 1px solid #ccc; margin-bottom: 5px; padding: 5px; background-color: ${color}">
                <b>ID:</b> ${m.mission_id} <br>
                <b>Status:</b> ${m.status} <br>
                <b>Robot:</b> ${m.assigned_robot_id || "Non assigné"} <br>
                <b>Objectif:</b> Waypoint ${m.goal_waypoint_idx}
            </div>
        `;
    });
    container.innerHTML = html;
}


// --- 4. EXPORTS ---
// Pour que le HTML puisse voir ces fonctions
window.initROS = initProject; // J'ai renommé en initProject pour être clair, update ton HTML body onload="initROS()"
window.uiManager = uiManager; // Pour le onchange="uiManager.toggle..."

window.addEventListener('resize', () => mapManager.resize());