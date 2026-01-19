class MapManager {
    constructor() {
        this.viewer = null;
        this.gridClient = null;
        this.robotMarker = null;
        
        // Conteneur pour les dessins de la mission (en Mètres)
        this.missionLayer = new createjs.Container();
        
        // --- CORRECTION RACINE DE L'AXE Y ---
        // On inverse l'échelle Y du conteneur. 
        // Ainsi, dessiner en Y positif "montera" visuellement sur l'écran.
        this.missionLayer.scaleY = -1; 
        
        this.isPicking = false;
        this.pickCallback = null;
    }

    init(ros, divId) {
        const el = document.getElementById(divId);
        
        // 1. Viewer
        this.viewer = new ROS2D.Viewer({
            divID: divId,
            width: el.offsetWidth,
            height: el.offsetHeight,
            background: '#0f0f0f'
        });

        // 2. Grid Client
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

        // 3. Ajout des Layers à la scène globale
        // On ajoute d'abord la mission (pour qu'elle soit sous le robot)
        this.viewer.scene.addChild(this.missionLayer);
        
        // On ajoute le robot
        this.robotMarker = new createjs.Shape();
        this.robotMarker.graphics.beginFill("#0088FF").drawPolyStar(0, 0, 0.3, 3, 0, -90);
        this.robotMarker.visible = false;
        this.viewer.scene.addChild(this.robotMarker);


        // 4. Picking (Clic Souris -> Mètres)
        // Note : le calcul du picking reste manuel car il part des pixels de l'écran.
        this.viewer.scene.addEventListener('stagemouseup', (evt) => {
            if (this.isPicking && this.pickCallback) {
                const grid = this.gridClient.currentGrid;
                if (!grid) return;

                var gridCoords = grid.globalToLocal(evt.stageX, evt.stageY);
                
                if (gridCoords.x >= 0 && gridCoords.x <= grid.image.width && 
                    gridCoords.y >= 0 && gridCoords.y <= grid.image.height) {
                    
                    const rosCoords = this.gridToRos(gridCoords.x, gridCoords.y);
                    this.pickCallback(rosCoords.x.toFixed(2), rosCoords.y.toFixed(2));
                    this.disablePicking();
                }
            }
        });
    }

    // --- CONVERSION (Uniquement pour le Clic) ---
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

    // --- AFFICHAGE (En Mètres) ---
    displayMissionPath(waypoints) {
        this.missionLayer.removeAllChildren();
        
        if (waypoints.length === 0) {
            this.viewer.scene.update();
            return;
        }

        const g = new createjs.Graphics();
        
        // --- NOUVEAU STYLE DE LIGNE ---
        // Trait fin (0.02m = 2cm), Gris, Pointillés
        // setStrokeDash([longueur_tiret, longueur_espace], offset)
        // Ici : tirets de 10cm espacés de 5cm
        g.setStrokeStyle(0.02).beginStroke("#888888").setStrokeDash([0.1, 0.05], 0);

        let start = true;
        // Taille de la demi-croix en mètres (ex: 0.15 = croix de 30cm de large)
        const crossSize = 0.15; 

        waypoints.forEach((wp) => {
            // On utilise DIRECTEMENT les coordonnées Mètres (ROS)
            // Grâce à missionLayer.scaleY = -1, plus besoin de se soucier de l'inversion ici.
            const mx = parseFloat(wp.x);
            const my = parseFloat(wp.y);

            // 1. Tracer le chemin en pointillés
            if(start) { g.moveTo(mx, my); start = false; }
            else { g.lineTo(mx, my); }

            // 2. Dessiner le point (Croix Bleue 'X')
            const cross = new createjs.Shape();
            // Style de la croix : Bleu, épaisseur 3cm, pas de pointillés
            cross.graphics.setStrokeStyle(0.03).beginStroke("#0000FF").setStrokeDash(null);
            
            // Première diagonale (\)
            cross.graphics.moveTo(-crossSize, crossSize).lineTo(crossSize, -crossSize);
            // Deuxième diagonale (/)
            cross.graphics.moveTo(-crossSize, -crossSize).lineTo(crossSize, crossSize);
            
            // Positionner la croix
            cross.x = mx;
            cross.y = my;
            
            // Comme le conteneur parent est inversé en Y, la croix serait dessinée à l'envers.
            // Pour un 'X' symétrique ça ne change rien, mais pour être rigoureux,
            // on ré-inverse l'échelle Y de cet objet spécifique pour qu'il soit "à l'endroit".
            cross.scaleY = -1; 

            this.missionLayer.addChild(cross);
        });

        const path = new createjs.Shape(g);
        this.missionLayer.addChildAt(path, 0); // Ligne sous les croix

        // Rafraichir
        this.viewer.scene.update();
    }

    enablePicking(callback) {
        this.isPicking = true;
        this.pickCallback = callback;
        document.body.style.cursor = 'crosshair';
    }

    disablePicking() {
        this.isPicking = false;
        this.pickCallback = null;
        document.body.style.cursor = 'default';
    }

    updateRobot(x, y, orientationQuaternion) {
        if(!this.robotMarker) return;
        this.robotMarker.visible = true;
        this.robotMarker.x = x;
        this.robotMarker.y = y;
        
        const q = orientationQuaternion;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);
        // Note : si le robot semble tourner à l'envers, il faudra peut-être retirer le signe '-' ici
        // selon la convention de votre odométrie.
        this.robotMarker.rotation = -yaw * (180.0 / Math.PI);
        
        this.viewer.scene.update();
    }

    resize() {
        const el = document.getElementById(this.viewer.divID);
        if(el && this.viewer) {
            this.viewer.resize(el.offsetWidth, el.offsetHeight);
            this.viewer.scene.update();
        }
    }
}