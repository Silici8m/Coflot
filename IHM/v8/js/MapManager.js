class MapManager {
    constructor() {
        this.viewer = null;
        this.gridClient = null;
        this.robotMarker = null;
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
                // Centrage de la vue
                this.viewer.scaleToDimensions(this.gridClient.currentGrid.width, this.gridClient.currentGrid.height);
                this.viewer.shift(this.gridClient.currentGrid.pose.position.x, this.gridClient.currentGrid.pose.position.y);
            }
        });

        this.robotMarker = new createjs.Shape();
        this.robotMarker.graphics.beginFill("#0088FF").drawPolyStar(0, 0, 0.5, 3, 0, -90);
        this.robotMarker.visible = false;
        this.viewer.scene.addChild(this.robotMarker);

        // --- ECOUTEUR DE CLIC (ALGORITHME CORRIGÉ) ---
        this.viewer.scene.addEventListener('stagemouseup', (evt) => {
            if (this.isPicking && this.pickCallback) {
                
                const grid = this.gridClient.currentGrid;
                if (!grid) {
                    console.warn("Pas de carte chargée !");
                    return;
                }

                // 1. Récupérer les coordonnées en PIXELS sur l'image de la carte
                var gridCoords = grid.globalToLocal(evt.stageX, evt.stageY);
                
                // 2. Récupérer les métadonnées de la carte
                // ros2djs stocke la résolution dans scaleX
                const resolution = grid.scaleX; 
                // Pour la hauteur en pixels, on utilise l'image source du bitmap
                const heightPixels = grid.image.height; 
                const originX = grid.pose.position.x;
                const originY = grid.pose.position.y;

                // 3. Vérifier qu'on a cliqué DANS l'image
                if (gridCoords.x < 0 || gridCoords.x > grid.image.width || 
                    gridCoords.y < 0 || gridCoords.y > heightPixels) {
                    console.warn("Clic hors de la carte");
                    return;
                }

                // 4. CALCUL ROS (Inversion de l'axe Y)
                // X = Origine + (Pixel * Resolution)
                const rosX = originX + (gridCoords.x * resolution);

                // Y = Origine + ((HauteurTotale - Pixel) * Resolution)
                // On part du bas de l'image (HauteurTotale) et on remonte
                const rosY = originY + ((heightPixels - gridCoords.y) * resolution);

                // 5. Envoi du résultat
                console.log(`[PICK] Pixel: (${gridCoords.x.toFixed(0)}, ${gridCoords.y.toFixed(0)}) -> ROS: (${rosX.toFixed(2)}, ${rosY.toFixed(2)})`);
                
                this.pickCallback(rosX.toFixed(2), rosY.toFixed(2));
                this.disablePicking();
            }
        });
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
        this.robotMarker.rotation = -yaw * (180.0 / Math.PI);
    }

    resize() {
        const el = document.getElementById(this.viewer.divID);
        if(el && this.viewer) this.viewer.resize(el.offsetWidth, el.offsetHeight);
    }
}