class MapManager {
    constructor() {
        this.viewer = null;
        this.gridClient = null;

        // Calque pour la mission
        this.missionLayer = new createjs.Container();
        this.missionLayer.scaleY = -1;

        // Calque pour la flotte
        this.fleetLayer = new createjs.Container();
        this.fleetLayer.scaleY = -1;

        // Dictionnaire pour stocker les marqueurs de la flotte
        this.fleetMarkers = {};
        
        this.isPicking = false;
        this.pickCallback = null;
    }

    init(ros, divId) {
        const el = document.getElementById(divId);

        this.viewer = new ROS2D.Viewer({
            divID: divId,
            width: el.offsetWidth,
            height: el.offsetHeight,
            background: '#0f0f0f'
        });

        this.gridClient = new ROS2D.OccupancyGridClient({
            ros: ros,
            rootObject: this.viewer.scene,
            continuous: false
        });

        this.gridClient.on('change', () => {
            if (this.gridClient.currentGrid) {
                this.viewer.scaleToDimensions(this.gridClient.currentGrid.width, this.gridClient.currentGrid.height);
                this.viewer.shift(this.gridClient.currentGrid.pose.position.x, this.gridClient.currentGrid.pose.position.y);
                this.viewer.scene.update();
            }
        });

        this.viewer.scene.addChild(this.missionLayer);
        this.viewer.scene.addChild(this.fleetLayer);
        

        this.viewer.scene.addEventListener('stagemouseup', (evt) => {
            if (this.isPicking && this.pickCallback) {
                const grid = this.gridClient.currentGrid;
                if (!grid) return;
                var gridCoords = grid.globalToLocal(evt.stageX, evt.stageY);
                if (gridCoords.x >= 0 && gridCoords.x <= grid.image.width && gridCoords.y >= 0 && gridCoords.y <= grid.image.height) {
                    const rosCoords = this.gridToRos(gridCoords.x, gridCoords.y);
                    this.pickCallback(rosCoords.x.toFixed(2), rosCoords.y.toFixed(2));
                    this.disablePicking();
                }
            }
        });
    }

    updateFleet(fleetMsg) {
        // fleetMsg contient : { robots: [ {robot_id, x, y, yaw, mode, ...}, ... ] }
        
        if(!fleetMsg.robots) return;

        fleetMsg.robots.forEach(robotState => {
            const id = robotState.robot_id;
            
            // 1. Récupérer ou créer le marqueur
            let marker = this.fleetMarkers[id];
            
            if (!marker) {
                // --- CRÉATION SI NOUVEAU ---
                marker = new createjs.Container();
                
                // Forme du robot
                const shape = new createjs.Shape();
                shape.name = "body";
                
                // On dessine initialement (sera redessiné plus bas de toute façon)
                this.drawRobotShape(shape, robotState.mode);
                
                marker.addChild(shape);
                
                // Ajout au calque Flotte
                this.fleetLayer.addChild(marker);
                this.fleetMarkers[id] = marker;
            }

            // 2. Mise à jour Position
            marker.x = robotState.x;
            marker.y = robotState.y;
            
            // 3. Mise à jour Rotation
            // Rappel : rotation négative car l'axe Y du canvas est souvent inversé visuellement
            marker.rotation = -robotState.yaw * (180.0 / Math.PI);

            // 4. Mise à jour Couleur / Forme
            // On récupère la forme existante et on la redessine pour être sûr 
            // que la couleur correspond au mode actuel (idle, charging, etc.)
            const shape = marker.getChildByName("body");
            this.drawRobotShape(shape, robotState.mode);
        });

        this.viewer.scene.update();
    }

    // Fonction utilitaire pour dessiner le cercle du robot
    drawRobotShape(shape, mode) {
        const radius = 0.15; // Rayon 15cm = Diamètre 30cm
        const color = this.getColorByMode(mode);

        shape.graphics.clear();

        // A. Bordure (pour mieux voir le robot sur la map sombre)
        shape.graphics.setStrokeStyle(0.02).beginStroke("white"); 

        // B. Corps du robot (Cercle de 30cm)
        shape.graphics.beginFill(color).drawCircle(0, 0, radius);

        // C. Indicateur de direction (Ligne blanche vers l'avant)
        // L'axe X (0 degrés) est vers la droite dans EaselJS
        shape.graphics.setStrokeStyle(0.03).beginStroke("white");
        shape.graphics.moveTo(0, 0).lineTo(radius, 0); // Trait du centre vers le bord avant
        
    }

    getColorByMode(mode) {
        switch(mode) {
            case 'idle': return '#057e43ff';     // Vert
            case 'busy': return '#0e4097ff';     // Bleu
            case 'charging': return '#a29607ff'; // Jaune
            case 'error': return '#a40a29ff';    // Rouge
            default: return '#9E9E9E';         // Gris
        }
    }

    gridToRos(pixelX, pixelY) {
        const grid = this.gridClient.currentGrid;
        const res = grid.scaleX;
        const originX = grid.pose.position.x;
        const originY = grid.pose.position.y;
        const heightPixels = grid.image.height;
        return {
            x: originX + (pixelX * res),
            y: originY + ((heightPixels - pixelY) * res)
        };
    }

    displayMissionPath(waypoints) {
        this.missionLayer.removeAllChildren();
        if (waypoints.length === 0) { this.viewer.scene.update(); return; }

        const g = new createjs.Graphics();
        g.setStrokeStyle(0.02).beginStroke("#888888").setStrokeDash([0.1, 0.05], 0);

        let start = true;
        const crossSize = 0.15; 

        waypoints.forEach((wp) => {
            const mx = parseFloat(wp.x);
            const my = parseFloat(wp.y);

            if(start) { g.moveTo(mx, my); start = false; }
            else { g.lineTo(mx, my); }

            const cross = new createjs.Shape();
            cross.graphics.setStrokeStyle(0.03).beginStroke("#0000FF").setStrokeDash(null);
            cross.graphics.moveTo(-crossSize, crossSize).lineTo(crossSize, -crossSize);
            cross.graphics.moveTo(-crossSize, -crossSize).lineTo(crossSize, crossSize);
            cross.x = mx;
            cross.y = my;
            cross.scaleY = -1; // Ré-inversion locale pour la croix
            this.missionLayer.addChild(cross);
        });

        const path = new createjs.Shape(g);
        this.missionLayer.addChildAt(path, 0);
        this.viewer.scene.update();
    }

    enablePicking(callback) { this.isPicking = true; this.pickCallback = callback; document.body.style.cursor = 'crosshair'; }
    disablePicking() { this.isPicking = false; this.pickCallback = null; document.body.style.cursor = 'default'; }

    resize() {
        const el = document.getElementById(this.viewer.divID);
        if(el && this.viewer) { this.viewer.resize(el.offsetWidth, el.offsetHeight); this.viewer.scene.update(); }
    }
}