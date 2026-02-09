# mission_manager

## 1 - Vue d'ensemble

Le package mission_manager est le nœud d'orchestration central du système de gestion de flotte "Coflot". Il a pour rôle fonctionnel de :

1. **Agréger l'état de la flotte** : Collecte les données de télémétrie des robots via un pool dynamique.
2. **Gérer le cycle de vie des missions** : Centralise les requêtes, le suivi des états et la validation des étapes.
3. **Allouer les tâches** : Exécute des algorithmes d'allocation (stratégies) pour assigner les robots aux missions de manière optimale.
4. **Publier l'état global** : Diffuse en temps réel l'état consolidé de toutes les missions.

### Arborescence des fichiers :
```
mission_manager/
├── allocation_strategies/       # Algorithmes d'attribution des missions
│   ├── allocation_interface.py 
│   ├── base_strategy.py
│   ├── closest_strategy.py
│   ├── naive_queue_strategy.py
│   └── utility_strategy.py
├── core/
│   ├── mission.py               # Machine à états d'une mission
│   ├── mission_registry.py      # Base de données locale des missions
│   ├── robot.py                 # Représentation d'un robot
│   ├── robot_adapter.py         # Interface avec les actions Nav 2 d'un robot
│   └── robot_pool.py            # Gestionnaire de la liste des robots
├── config.py                    # Paramètres globaux et constantes
├── mission_manager_node.py      # Point d'entrée du nœud ROS 2
├── package.xml
└── setup.py
```

## 2 - Utilisation et Lancement

Ce package ne contient pas de dossier launch/ dans les sources fournies. Le lancement s'effectue via l'exécutable `mission_manager`.

### Exécutables (ros2 run)

La commande suivante permet de démarrer le gestionnaire de missions :

```bash
ros2 run mission_manager mission_manager
```

### Arguments et Paramètres

Le nœud accepte des paramètres ROS 2 standards (via --ros-args). Le paramètre `allocation_strategy` configure la stratégie d'allocation à utiliser au démarrage.

| Stratégie (ID) | Type d'Algorithme | Fonctionnement & Caractéristiques |
| :--- | :--- | :--- |
| **`utility`** *(Défaut)* | Optimisation Globale (Hongrois) | Maximise l'utilité globale (`Qualité - Coût`). Calcule le temps de trajet réel via le planificateur Nav2, inclut les pénalités de transition et gère la préemption pour les missions urgentes. |
| `closest` | Glouton (Greedy) | Minimise la distance euclidienne (vol d'oiseau) entre le robot disponible et le premier point de la mission. Ne considère pas la structure de l'environnement (obstacles). |
| `simple_queue` | FIFO (First-In-First-Out) | Approche naïve. Assigne la mission la plus ancienne et prioritaire au premier robot qui se libère, sans aucune optimisation spatiale. |

Lancement du gestionnaire de mission avec la stratégie `closest` : 

```bash
ros2 run mission_manager mission_manager --ros-args -p allocation_strategy:=closest
```

## 3 - Interfaces ROS

### Topics Souscrits (Subscribers)

| Topic | Type de message | Utilité |
| :--- | :--- | :--- |
| `/fleet/fleet_state` | `fleet_interfaces/RobotStateArray` | Réception de la télémétrie (pose, batterie) de tous les robots actifs pour mise à jour du `RobotPool`. |
| `/mission/mission_request` | `fleet_interfaces/MissionRequest` | Réception des nouvelles demandes de missions (ID, priorité, waypoints). |
| `/mission/validation` | `std_msgs/String` | Signal de validation manuelle (opérateur humain) contenant l'ID de la mission. |


### Topics Publiés (Publishers)

| Nom du topic | Type de message | Condition de publication |
| :--- | :--- | :--- |
| `/mission/missions_state` | `fleet_interfaces/MissionStateArray` | Publié périodiquement (**0.2s**) ou lors de changements d'états. QoS configuré en **Transient Local** (Latched). |

### Actions Client

Le nœud agit comme un Client d'action pour contrôler les robots. Les noms des actions sont dynamiques et dépendent de l'ID du robot ({robot_id}).

| Nom | Type | Description |
| :--- | :--- | :--- |
| `/{robot_id}/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Envoi des ordres de déplacement du robot vers un waypoint. |
| `/{robot_id}/compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | Demande de calcul de chemin (sans déplacement) pour estimer les coûts de trajet dans la stratégie **Utility**. |

### Paramètres

| Nom | Type | Valeur Défaut | Description |
| :--- | :--- | :--- | :--- |
| `allocation_strategy` | `string` | `'utility'` | Sélectionne l'algorithme d'allocation à utiliser. Choix : `'utility'`, `'closest'`, `'simple_queue'`. |

## 4 - Architecture et Logique

### Machine à États (Mission Lifecycle)

Chaque mission est gérée par une machine à états finis définie dans `mission.py`. Voici le diagramme des transitions possibles :


```mermaid
stateDiagram-v2
    [*] --> PENDING
    
    PENDING --> ASSIGNED : Assign_robot
    ASSIGNED --> APPROACHING : Start
    
    %% Phase d'approche (Robot vide)
    APPROACHING --> WAITING : Arrive
    APPROACHING --> PENDING : Revoke
    
    %% Boucle de livraison
    WAITING --> DELIVERING : Continue
    DELIVERING --> WAITING : Arrive
    
    %% Séquence de Suspension (L'ajout demandé)
    WAITING --> SUSPENDING : Suspend
    SUSPENDING --> DISCHARGING : Discharge
    DISCHARGING --> PENDING : Release
    
    %% Fin standard
    WAITING --> FINISHED : Finish
    FINISHED --> [*]
    
    %% Echecs
    APPROACHING --> FAILED : Fail
    DELIVERING --> FAILED : Fail
    FAILED --> [*]
```


[Schéma cycle de vie d'une mission](https://drive.google.com/file/d/1-l7DNsJttKJm8WjJinMwKRsExWfNRGUP/view?usp=drive_link)
