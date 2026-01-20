class UIManager {
    constructor() {
        this.waypointCount = 0;
        this.container = document.getElementById('waypointsContainer');
    }

    updateStatus(isConnected) {
        const ind = document.getElementById('status-indicator');
        if (ind) {
            ind.innerHTML = isConnected ? "🟢 Connecté" : "🔴 Déconnecté / Erreur";
            ind.style.color = isConnected ? "#00c853" : "red";
        }
    }

    updateRobotInfo(x) {
        const el = document.getElementById('robot-info');
        if (el) el.innerHTML = `X: ${x.toFixed(2)}`;
    }

    // Ajoute un bloc HTML waypoint
    addWaypoint() {
        this.waypointCount++;
        
        // Génération des options à partir des données
        let zoneOptions = PRESET_ZONES.map(z => `<option value="${z.id}">${z.name}</option>`).join('');

        const html = `
        <div class="waypoint-card" id="wp-${this.waypointCount}">
            <div class="wp-row-1">
                <select class="wp-select-type" onchange="uiManager.toggleCoordInput(${this.waypointCount}, this)">
                    <option value="custom">Personnalisé</option>
                    ${zoneOptions}
                </select>
                <button class="btn-icon btn-save" onclick="savePoint(${this.waypointCount})"><img src="save.svg"></button>
                <button class="btn-icon btn-delete-db" onclick="deleteFromDB(${this.waypointCount})"><img src="delete.svg"></button>
                <button class="btn-icon btn-pick" onclick="startPickMap(${this.waypointCount})"><img src="cible.svg"></button>
                <button class="btn-icon btn-clear" onclick="removeWaypointUI(${this.waypointCount})"><img src="clear.svg"></button>
            </div>
            <div class="wp-row-2">
                <span class="wp-coord-label">X:</span>
                <input type="text" class="wp-coord" name="x" value="0.00" oninput="updateMapViz()">
                <span class="wp-coord-label">Y:</span>
                <input type="text" class="wp-coord" name="y" value="0.00" oninput="updateMapViz()">
            </div>
        </div>`;
        
        this.container.insertAdjacentHTML('beforeend', html);
        this.container.scrollTop = this.container.scrollHeight;
    }
    removeWaypoint(id) {
        const el = document.getElementById(`wp-${id}`);
        if (el) el.remove();
    }

    // Active/Désactive les inputs selon le select
    toggleCoordInput(id, selectEl) {
        const card = document.getElementById(`wp-${id}`);
        const inputX = card.querySelector('input[name="x"]');
        const inputY = card.querySelector('input[name="y"]');
        
        if (selectEl.value !== 'custom') {
            const zone = PRESET_ZONES.find(z => z.id === selectEl.value);
            if (zone) {
                inputX.value = zone.x.toFixed(2);
                inputY.value = zone.y.toFixed(2);
                inputX.disabled = true;
                inputY.disabled = true;
                inputX.style.opacity = '0.5';
                inputY.style.opacity = '0.5';
            }
        } else {
            inputX.disabled = false;
            inputY.disabled = false;
            inputX.style.opacity = '1';
            inputY.style.opacity = '1';
        }
        window.updateMapViz(); // Met à jour le dessin des traits immédiatement
    }

    // Remplit les champs d'un waypoint spécifique (utilisé par le Picking)
    setWaypointCoords(id, x, y) {
        const card = document.getElementById(`wp-${id}`);
        if (card) {
            card.querySelector('input[name="x"]').value = x;
            card.querySelector('input[name="y"]').value = y;
            // Force le mode "custom"
            const sel = card.querySelector('.wp-select-type');
            sel.value = 'custom';
            this.toggleCoordInput(id, sel);
        }
    }
    
    getAllWaypoints() {
        const waypoints = [];
        const wpElements = document.querySelectorAll('.waypoint-card');
        
        wpElements.forEach(card => {
            const xStr = card.querySelector('input[name="x"]').value;
            const yStr = card.querySelector('input[name="y"]').value;
            
            const xVal = parseFloat(xStr);
            const yVal = parseFloat(yStr);

            if (!isNaN(xVal) && !isNaN(yVal)) {
                // Structure stricte geometry_msgs/Pose
                const pose = {
                    position: {
                        x: xVal,
                        y: yVal,
                        z: 0.0 // Toujours 0 en 2D, mais obligatoire
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0 // Quaternion neutre (pas de rotation)
                    }
                };
                waypoints.push(pose);
            }
        });
        return waypoints;
    }

    getName() {
        const el = document.getElementById('missionName');
        return el ? el.value : null;
    }

    getPriority() {
        const el = document.querySelector('input[name="prio"]:checked');
        return el ? el.value : 1;
    }
}