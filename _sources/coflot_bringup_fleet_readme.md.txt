# coflot_bringup_fleet

## 1 - Vue d'ensemble

Le package `coflot_bringup_fleet` contient un fichier de lancement pour connecter l'Interface Homme Machine du projet Coflot.

### Rôle fonctionnel

1. **Lancement du serveur de Carte** : Chargement et publication de la carte statique via `nav2_map_server`.
2. **Passerelle de Communication** : Ouverture d'un port WebSocket via `rosbridge_server` pour permettre la connexion de l'IHM.


### Arborescence

```
coflot_bringup_fleet/
├── launch/
│   └── bring_up.launch.py    # Script de lancement
├── maps/
│   ├── salle2.pgm            # Carte
│   └── salle2.yaml           # Métadonnées de la carte
├── package.xml
└── setup.py
```

## 2 - Utilisation et Lancement

### Launch Files (ros2 launch)

Le lancement de l'infrastructure s'effectue via la commande suivante :

```bash
ros2 launch coflot_bringup_fleet bring_up.launch.py
```

### Arguments de lancement

| Argument | Valeur Défaut | Description |
| :--- | :--- | :--- |
| `map` | `.../maps/salle2.yaml` | Chemin absolu vers le fichier de configuration YAML de la carte à charger. |


## 3 - Infrastructure Déployée

Le fichier `bring_up.launch.py` démarre les trois processus suivants. Les commandes Bash ci-dessous permettent de lancer ces nœuds individuellement pour le débogage (équivalence stricte avec les paramètres du launch file).

### 1. Serveur de Carte (`nav2_map_server`)

- **Rôle :** Charge le fichier de carte statique (OccupancyGrid) et le publie sur le réseau.
- **Commande équivalente :**
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/chemin/vers/salle2.yaml -p frame_id:=map
```

### 2. Gestionnaire de Cycle de Vie (`nav2_lifecycle_manager`)

- **Rôle :** Supervise le nœud `map_server`. Il effectue automatiquement la transition de l'état *Unconfigured* vers *Active* pour que la carte soit effectivement publiée.
- **Commande équivalente :**
```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=True -p node_names:=['map_server']
```

### 3. Serveur Rosbridge (`rosbridge_server`)

- **Rôle :** Crée une passerelle WebSocket (TCP) permettant aux applications externes (Web, IHM) d'interagir avec ROS 2 via JSON.
- **Commande équivalente :**
```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

