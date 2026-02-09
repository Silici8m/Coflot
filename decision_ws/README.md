# Coflot - Système de Gestion de Flotte de Robots

Ce workspace contient les composants décisionnels et de simulation pour la gestion d'une flotte de robots mobiles sous **ROS 2 Humble**.

## 1 - Système et Prérequis

### Environnement Logiciel

- **OS :** Ubuntu 22.04 LTS
- **Version ROS :** ROS 2 Humble
- **Python :** Python 3.10+

### Dépendances Système
```bash 
sudo apt update 
sudo apt install python3-pip python3-colcon-common-extensions ros-humble-nav2-msgs ros-humble-geometry-msgs
```

### Environnement Virtuel (Pour documentation)

Pour la génération de la documentation Sphinx : 
```bash
bash python3 -m venv .venv-doc source .venv-doc/bin/activate pip install sphinx myst-parser sphinxcontrib-mermaid sphinx-rtd-theme sphinx-autodoc-typehints
```

## 2 - Compilation du Workspace

### Compilation

Utilisez `colcon` pour compiler l'ensemble des packages du projet.

```bash
cd ~/Coflot/decision_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note :** À la première compilation, fleet_interfaces peut générer un avertissement dû à l'ordre de dépendance. Une seconde compilation résout généralement le problème.

### Sourcing de l'environnement

```bash
source install/setup.bash
```

## 3 - Architecture des Packages

- `fleet_adapter` : Interface entre le gestionnaire de missions et les robots individuels.
- `mission_manager` : Cœur logique gérant l'allocation et le cycle de vie des missions.
- `fleet_simulation` : Simulateur léger pour tester les algorithmes sans hardware.
- `coflot_bringup_fleet` : Scripts de lancement et configurations globales.
- `fleet_interfaces` : Définitions des messages (`.msg`) personnalisés.

## 4 - Commandes de Lancement

### Gestionnaire de mission (`mission_manager`)

La commande suivante permet de lancer le gestionnaire de mission, le paramètre `allocation_strategy` permet de définir la stratégie d'allocation des missions aux robots. (`utility`(*par défaut*), `closest`, `naive_queue`)

```bash
source install/setup.bash
ros2 run mission_manager mission_manager --ros-args -p allocation_strategy:=utility
```

### Lancement de l'adaptateur de flotte **réel** (`fleet_adapter`)

La commande suivante permet de lancer le gestionnaire de mission, qui supervise les robots connecté et publie leur état sur `/fleet/robots_state`.

```bash
source install/setup.bash
ros2 run fleet_adapter fleet_adapter
```

### Lancement de la **simulation** de flotte (`fleet_simulation`)

La commande suivante permet de lancer le simulateur de flotte.
Ce noeud remplace le `fleet_adapter` et créé une flotte virtuelle.
```bash 
ros2 run fleet_simulation fleet_simulation
```

### Lancement de la communication IHM (`coflot_bringup_fleet`)

La commande suivante permet de lancer le programme qui gère les échanges de données avec l'IHM.
```bash 
ros2 launch coflot_bringup_fleet bringup.launch.py
```

### Ouverture de l'IHM

L'**IHM** est située dans le dossier *IHM* à la racine du projet. Ouvrez le fichier `index.html` dans un navigateur pour superviser la flotte en temps réel.


## 5 - Documentation

La documentation technique est générée via **Sphinx**. Elle agrège les fichiers sources, les README de chaque package, les définitions des messages ROS et la documentation de l'API Python.

### Installation de l'environnement virtuel Python
Un environnement virtuel Python est nécessaire pour installer les outils de documentation sans polluer le système.

```bash
cd docs
# Création de l'environnement virtuel
python3 -m venv .venv-doc
# Activation
source .venv-doc/bin/activate
# Installation des dépendances
pip install -r requirements.txt
```

### Génération de la documentation

Le script `build_docs.sh` automatise le nettoyage, la copie des fichiers sources (README, messages, launch files) et la compilation HTML.


```bash
cd docs
./build_docs.sh
```

### Consultation

Une fois la génération terminée, ouvrez le fichier suivant dans votre navigateur : `docs/build/html/index.html`

