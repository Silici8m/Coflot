// Connexion au serveur rosbridge
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090' // Assure-toi que rosbridge est lancé
});

ros.on('connection', () => console.log("✅ Connecté à ROS2 via rosbridge"));
ros.on('error', e => console.error("Erreur rosbridge:", e));
ros.on('close', () => console.log("❌ Connexion fermée"));

// Définir le topic Mission
const missionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/mission_topic',
    messageType: 'my_fleet_manager/msg/Mission'
});

// Gestion de la carte
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
let points = [];
let missionId = 0;
let mapImage = new Image();
let mapLoaded = false;

// --- paramètres de ta carte depuis map.yaml ---
const mapConfig = {
    resolution: 0.05,
    origin: [-3.75, -3.04, 0],
    path: 'map/map.png' // <-- PNG maintenant
  };

// Charger et afficher la carte
mapImage.onload = () => {
    console.log("🗺️ Carte chargée :", mapConfig.path);
    canvas.width = mapImage.width;
    canvas.height = mapImage.height;
    ctx.drawImage(mapImage, 0, 0    ); //
    mapLoaded = true;
};
mapImage.onerror = () => console.error("❌ Impossible de charger la carte :", mapConfig.path);
mapImage.src = mapConfig.path;

// Fonction de conversion (pixels -> coordonnées ROS)
function canvasToRos(x, y) {
    const rosX = x * mapConfig.resolution + mapConfig.origin[0];
    const rosY = (canvas.height - y) * mapConfig.resolution + mapConfig.origin[1];
    return { x: rosX, y: rosY };
}

// Redessiner la carte et les points
function redraw() {
    if (mapLoaded) ctx.drawImage(mapImage, 0, 0);
    else ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.strokeRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = 'blue';
    ctx.font = '14px Arial';
    points.forEach((p, i) => {
        ctx.beginPath();
        ctx.arc(p.cx, p.cy, 5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillText(i + 1, p.cx + 8, p.cy - 8);
    });
}

// Clique sur la carte → ajouter un point
canvas.addEventListener('click', e => {
    if (!mapLoaded) {
        alert("La carte n'est pas encore chargée !");
        return;
    }

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const rosPos = canvasToRos(cx, cy);

    points.push({ cx, cy, x: rosPos.x, y: rosPos.y });
    redraw();

    document.getElementById('pointsList').innerText =
        points.map((p, i) => `Point ${i + 1}: (${p.x.toFixed(2)}, ${p.y.toFixed(2)})`).join('\n');
});

// Envoyer la mission
document.getElementById('sendBtn').addEventListener('click', () => {
    if (points.length === 0) {
        alert("Sélectionne au moins un point !");
        return;
    }

    missionId++;
    const missionMsg = new ROSLIB.Message({
        mission_id: missionId,
        num_points: points.length,
        points: points.map(p => ({
            position: { x: p.x, y: p.y, z: 0.0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }))
    });

    missionTopic.publish(missionMsg);
    console.log("✅ Mission envoyée :", missionMsg);
    alert(`Mission ${missionId} envoyée avec ${points.length} points`);
    points = [];
    redraw();
    document.getElementById('pointsList').innerText = "";
});
