// --- 1. INSTANCIATION ---
const rosManager = new RosManager();
const mapManager = new MapManager();
const uiManager  = new UIManager();

// --- 2. INIT GLOBAL (Appelé par body onload) ---
function initProject() {
    // A. Connexion
    rosManager.connect(
        () => { // Succès
            uiManager.updateStatus(true);
            
            // On lance la map une fois connecté
            mapManager.init(rosManager.ros, 'map-panel');
            mapManager.drawZones();

            // On s'abonne au robot
            rosManager.subscribeRobotPose((x, y, orientation) => {
                mapManager.updateRobot(x, y, orientation);
                uiManager.updateRobotInfo(x);
            });
        },
        () => { // Erreur
            uiManager.updateStatus(false);
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
    const prio = uiManager.getPriority();
    const points = uiManager.getAllWaypoints();
    rosManager.sendMission(prio, points);
};

// --- 4. EXPORTS ---
// Pour que le HTML puisse voir ces fonctions
window.initROS = initProject; // J'ai renommé en initProject pour être clair, update ton HTML body onload="initROS()"
window.uiManager = uiManager; // Pour le onchange="uiManager.toggle..."

window.addEventListener('resize', () => mapManager.resize());