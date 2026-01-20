class MapManager {
    constructor() {
        this.viewer = null;
        this.gridClient = null;

        // --- CALQUES (Mode Mètres, Y inversé) ---
        this.previewLayer = new createjs.Container();
        this.previewLayer.scaleY = -1; 
        
        this.activeMissionsLayer = new createjs.Container();
        this.activeMissionsLayer.scaleY = -1;

        this.fleetLayer = new createjs.Container();
        this.fleetLayer.scaleY = -1;

        this.zonesLayer = new createjs.Container();
        this.zonesLayer.scaleY = -1;

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
            background: '#121212' 
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
                
                
                // On force l'ordre des calques (Map en dessous, le reste au dessus)
                this.viewer.scene.addChild(this.activeMissionsLayer);
                this.viewer.scene.addChild(this.previewLayer);
                this.viewer.scene.addChild(this.fleetLayer);
                this.viewer.scene.addChild(this.zonesLayer);
                this.drawZones();
                
                this.viewer.scene.update();
            }
        });

        this.viewer.scene.addChild(this.activeMissionsLayer);
        this.viewer.scene.addChild(this.previewLayer);
        this.viewer.scene.addChild(this.fleetLayer);
        this.viewer.scene.addChild(this.zonesLayer);

        // Picking
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

    // =========================================================
    //  NOUVELLE LOGIQUE DE DESSIN (CLEAN & MINIMALISTE)
    // =========================================================

    drawPath(layer, waypoints, style) {
        if (style.clearLayer) layer.removeAllChildren();
        if (!waypoints || waypoints.length === 0) { this.viewer.scene.update(); return; }

        const g = new createjs.Graphics();
        
        // 1. LES LIAISONS (Très discrètes)
        // Trait fin et semi-transparent pour ne pas "barrer" la carte
        g.setStrokeStyle(0.02); // 2cm d'épaisseur
        g.beginStroke(style.lineColor || style.color);
        
        if (style.dashArray) g.setStrokeDash(style.dashArray, 0);

        let start = true;
        // On dessine d'abord toutes les lignes
        waypoints.forEach(wp => {
            let mx, my;
            if (wp.position) { mx = wp.position.x; my = wp.position.y; } 
            else { mx = parseFloat(wp.x); my = parseFloat(wp.y); }
            
            if (isNaN(mx) || isNaN(my)) return;

            if (start) { g.moveTo(mx, my); start = false; } 
            else { g.lineTo(mx, my); }
        });
        
        const pathShape = new createjs.Shape(g);
        pathShape.alpha = 0.4; // Transparence forte sur les lignes (40% visible)
        layer.addChild(pathShape);

        // 2. LES WAYPOINTS (Points bien visibles)
        waypoints.forEach((wp, idx) => {
            let mx, my;
            if (wp.position) { mx = wp.position.x; my = wp.position.y; } 
            else { mx = parseFloat(wp.x); my = parseFloat(wp.y); }

            if (isNaN(mx) || isNaN(my)) return;

            const isTarget = (style.highlightIndex !== undefined && idx === style.highlightIndex);
            this.drawWaypoint(layer, mx, my, style.color, isTarget);
        });
        
        this.viewer.scene.update();
    }

    // Dessine un point (Cercle) au lieu d'une croix
    drawWaypoint(layer, x, y, color, isTarget) {
        const shape = new createjs.Shape();
        
        if (isTarget) {
            // CIBLE ACTUELLE : Gros point plein avec anneau
            // Anneau extérieur
            shape.graphics.setStrokeStyle(0.02).beginStroke(color).drawCircle(0, 0, 0.20); // 20cm
            // Point central plein
            shape.graphics.beginFill(color).drawCircle(0, 0, 0.10); // 10cm
            shape.alpha = 1.0;
        } else {
            // POINT STANDARD : Petit point discret
            shape.graphics.beginFill(color).drawCircle(0, 0, 0.08); // 8cm
            shape.alpha = 0.8;
        }

        shape.x = x;
        shape.y = y;
        // Pas besoin de scaleY ici pour un cercle, mais bonne pratique
        shape.scaleY = -1; 
        
        layer.addChild(shape);
    }

    // =========================================================
    //  COMMANDES D'AFFICHAGE
    // =========================================================

    displayMissionPath(waypoints) {
        // STYLE PREVIEW : Cyan électrique, pointillés
        const style = {
            color: "#1e525fff",      // Cyan très visible sur noir
            lineColor: "rgba(65, 117, 122, 1)",
            dashArray: [0.1, 0.1], // Pointillés
            clearLayer: true
        };
        this.drawPath(this.previewLayer, waypoints, style);
    }

    updateMissions(missions) {
        this.activeMissionsLayer.removeAllChildren();
        if (!missions) { this.viewer.scene.update(); return; }

        missions.forEach(m => {

            // STYLE MISSION ACTIVE : Violet/Magenta (distinct du Cyan)
            const style = {
                color: "#9c7d00ff",       // Magenta vif
                lineColor: "#ba8c00ff",   // Ligne un peu plus sombre
                dashArray: null,        // Ligne continue
                clearLayer: false,
                highlightIndex: m.goal_waypoint_idx
            };
            
            // Si le robot est assigné, on peut changer la couleur légèrement (ex: Vert)
            if (m.assigned_robot_id) {
                style.color = "#004507ff"; // Vert si ça roule
                style.lineColor = "#rgba(0, 156, 0, 0.59)";
            }

            this.drawPath(this.activeMissionsLayer, m.waypoints, style);
        });
        this.viewer.scene.update();
    }

    // =========================================================
    //  ROBOTS (TRIANGLES ORIENTÉS)
    // =========================================================

    updateFleet(fleetMsg) {
        if(!fleetMsg.robots) return;

        fleetMsg.robots.forEach(robot => {
            let marker = this.fleetMarkers[robot.robot_id];
            
            if (!marker) {
                marker = new createjs.Container();
                
                // Forme du robot
                const shape = new createjs.Shape();
                shape.name = "body";
                marker.addChild(shape);

                // Texte ID (discret)
                const text = new createjs.Text(robot.robot_id, "0.2px Arial", "#FFF");
                text.y = -0.4; 
                text.textAlign = "center";
                text.scaleY = -1;
                text.shadow = new createjs.Shadow("#000", 0, 0, 0.1);
                marker.addChild(text);
                
                this.fleetLayer.addChild(marker);
                this.fleetMarkers[robot.robot_id] = marker;
            }

            // 1. POSITION
            marker.x = robot.pose.x;
            marker.y = robot.pose.y;

            // 2. ORIENTATION (Quaternion -> Yaw)
            const q = robot.pose.orientation;
            if (q) {
                // Conversion Quaternion vers Angle (Radians)
                // Formule standard : atan2(2(wz + xy), 1 - 2(y² + z²))
                // Pour ROS (Z-up), c'est souvent atan2(2(wy + zx), 1 - 2(y² + z²)) mais attention aux axes
                // Formule robuste Yaw (Z-axis rotation):
                const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                const yaw = Math.atan2(siny_cosp, cosy_cosp);

                // Conversion Radians -> Degrés pour EaselJS
                // Note : scaleY = -1 sur le container inverse visuellement la rotation.
                // -yaw * 180/PI fonctionne généralement bien avec ROS2Djs et scaleY=-1
                marker.rotation = -yaw * (180.0 / Math.PI);
            }

            // 3. COULEUR & FORME
            const shape = marker.getChildByName("body");
            this.drawRobotShape(shape, robot.mode);
        });
        this.viewer.scene.update();
    }

    drawRobotShape(shape, mode) {
        const color = this.getColorByMode(mode);
        shape.graphics.clear();
        
        // TRIANGLE TYPE "FLÈCHE DE NAVIGATION"
        // La pointe est vers X+ (0.3m devant), la base est large à l'arrière
        const length = 0.3; // 30cm devant
        const back = -0.15; // 15cm derrière
        const width = 0.20; // 20cm de large (aile)

        shape.graphics.setStrokeStyle(0.02).beginStroke("white").beginFill(color);
        
        // Pointe avant (X+, Y0) -> Aile Droite (X-, Y+) -> Creux Arrière (X-, Y0) -> Aile Gauche (X-, Y-) -> Fin
        shape.graphics.moveTo(length, 0);
        shape.graphics.lineTo(back, width);
        shape.graphics.lineTo(back + 0.05, 0); // Petit creux stylé à l'arrière
        shape.graphics.lineTo(back, -width);
        shape.graphics.closePath();
    }

    getColorByMode(mode) {
        switch(mode) {
            case 'idle': return '#198754';     // Vert Foncé
            case 'busy': return '#0D6EFD';     // Bleu
            case 'charging': return '#D97706'; // Orange
            case 'error': return '#DC3545';    // Rouge
            default: return '#757575';         // Gris
        }
    }

    enablePicking(callback) { this.isPicking = true; this.pickCallback = callback; document.body.style.cursor = 'crosshair'; }
    disablePicking() { this.isPicking = false; this.pickCallback = null; document.body.style.cursor = 'default'; }
    resize() {
        const el = document.getElementById(this.viewer.divID);
        if(el && this.viewer) { this.viewer.resize(el.offsetWidth, el.offsetHeight); this.viewer.scene.update(); }
    }

    drawZones() {
        this.zonesLayer.removeAllChildren();
        
        PRESET_ZONES.forEach(zone => {
            const container = new createjs.Container();
            container.x = zone.x;
            container.y = zone.y;

            const w = zone.w || 0.5;
            const h = zone.h || 0.5;

            // Rectangle rempli
            const box = new createjs.Shape();
            box.graphics.beginFill(zone.color).drawRect(-w/2, -h/2, w, h);
            box.alpha = 0.15;
            
            // Bordure
            const border = new createjs.Shape();
            border.graphics.setStrokeStyle(0.02).beginStroke(zone.color).drawRect(-w/2, -h/2, w, h);

            // Texte à l'intérieur (centré)
            const text = new createjs.Text(zone.name.toUpperCase(), "bold 0.12px Arial", zone.color);
            text.textAlign = "center";
            text.textBaseline = "middle";
            text.scaleY = -1; // Inversion pour compenser le layer
            text.alpha = 0.8;

            container.addChild(box, border, text);
            this.zonesLayer.addChild(container);
        });
        this.viewer.scene.update();
    }
}