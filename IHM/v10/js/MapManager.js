class MapManager {
    constructor() {
        this.viewer = null;
        this.gridClient = null;
        this.robotMarker = null;
        this.missionLayer = new createjs.Container();
        
        // CORRECTION AXE Y
        this.missionLayer.scaleY = -1; 
        
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
        
        this.robotMarker = new createjs.Shape();
        this.robotMarker.graphics.beginFill("#0088FF").drawPolyStar(0, 0, 0.3, 3, 0, -90);
        this.robotMarker.visible = false;
        this.viewer.scene.addChild(this.robotMarker);

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
    
    updateRobot(x, y, q) {
        if(!this.robotMarker) return;
        this.robotMarker.visible = true;
        this.robotMarker.x = x;
        this.robotMarker.y = y;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        this.robotMarker.rotation = -(Math.atan2(siny_cosp, cosy_cosp)) * (180.0 / Math.PI);
        this.viewer.scene.update();
    }

    resize() {
        const el = document.getElementById(this.viewer.divID);
        if(el && this.viewer) { this.viewer.resize(el.offsetWidth, el.offsetHeight); this.viewer.scene.update(); }
    }
}