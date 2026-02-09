# fleet_adapter

## 1 - Vue d'ensemble

Le package fleet_adapter agit comme un nœud d'agrégation d'état au sein de l'écosystème Coflot. 
### Rôle fonctionnel
Le nœud assure les fonctions suivantes :
1. **Suivi des connexions** : Détecte dynamiquement l'arrivée et le départ des robots sur le réseau ROS.
2. **Surveillance des états** : Centralise les données de télémétrie de chaque robot connecté (niveau de batterie, position, statut).
3. **Diffusion globale** : Concatener ces informations pour publier l'état unifié de la flotte sur le topic `/fleet/fleet_state`.

### Arborescence 

```
fleet_adapter/
├── fleet_adapter/
│   └── fleet_adapter.py    # Nœud principal (FleetStateAggregator)
├── package.xml
└── setup.py
```

## 2 - Utilisation et Lancement

Le lancement s'effectue via l'exécutable `fleet_adapter`. 

### Exécutables (ros2 run)

La commande pour démarrer le nœud `fleet_adapter` est :

```bash
ros2 run fleet_adapter fleet_adapter
```

## 3 - Interfaces ROS

Le noeud `fleet_adapter` utilise un mécanisme de souscription dynamique. Il scanne le graphe ROS pour détecter les robots actifs et crée des souscriptions à la volée.

### Topics Souscrits (Subscribers)

Le noeud `fleet_adapter` s'abonne aux topics ci-dessous pour chaque robot détecté.
> **Note :** `{robot_id}` correspond à l'identifiant du robot (namespace).

| Nom du topic | Type de message | Utilité |
| :--- | :--- | :--- |
| `/{robot_id}/sensors/battery_state` | `sensor_msgs/BatteryState` | Surveillance du niveau de batterie (pourcentage ou voltage). |
| `/{robot_id}/behavior_tree_log` | `nav2_msgs/BehaviorTreeLog` | Suivi du statut de navigation (IDLE, RUNNING, etc.) via les logs du Behavior Tree. |
| `/{robot_id}/robot_pose` | `geometry_msgs/Pose` | Récupération de la position et de l'orientation actuelles du robot. |

### Topics Publiés (Publishers)

| Nom du topic | Type de message | Condition de publication |
| :--- | :--- | :--- |
| `/fleet/fleet_state` | `fleet_interfaces/RobotStateArray` | Publié périodiquement à **10 Hz**. Contient la liste consolidée des états de tous les robots détectés. |

## 4 - Architecture et Logique

### Logique de Découverte Automatique

Le nœud ne nécessite pas de liste de robots statique. Il utilise une boucle de balayage (`scan_for_robots`) exécutée toutes les **1.0 seconde** :
1. Le nœud récupère la liste de tous les topics actifs du système via `get_topic_names_and_types`.
2. Il recherche les topics se terminant par `/sensors/battery_state`.
3. Le préfixe du topic est extrait et considéré comme le `robot_id` (ex: pour `/robot_1/sensors/battery_state`, l'ID est `robot_1`).
4. Si un nouvel ID est trouvé, le robot est enregistré et les souscriptions (Batterie, BT Log, Pose) sont instanciées.
5. Si un robot connu ne publie plus sur ses topics, il est désenregistré de la liste interne.

### Gestion des Données et Conversion

Les données reçues sont stockées dans un dictionnaire interne `robots_data`. Le nœud applique des traitements spécifiques avant la republication :
- **Batterie** : Le callback `battery_callback` gère deux formats :
    - Si `percentage` est valide (<= 1.0), il est converti en pourcentage (0-100).
    - Si `percentage` est absent (`NaN`) mais que `voltage` est présent, une conversion linéaire est appliquée en supposant une plage de **12.0V (0%) à 16.5V (100%)**.
- **Mode/Statut** : Le statut est dérivé du dernier événement du `BehaviorTreeLog`. Si le statut est "RUNNING", le mode devient "busy", sinon "idle".
- **Surveillance de la Latence (Timeout) :**
    - Lors de la publication de l'état global, le nœud vérifie l'âge de la dernière position reçue (`last_pose_time`).
    - Si la dernière donnée date de plus de **2.0 secondes** (`POSE_TIMEOUT_SEC`), un avertissement (WARN) est émis dans les logs : *"⚠️ Pas de données de pose pour {robot_id}..."*.