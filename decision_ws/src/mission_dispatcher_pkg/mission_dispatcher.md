# SPÉCIFICATIONS FONCTIONNELLES : GESTION DU CYCLE DE VIE DES MISSIONS

Ce document décrit les interactions fonctionnelles et les transitions d'états du nœud `Mission_Dispatcher`.

## 1. ACTIONS DE PILOTAGE (COMMANDES)
*Ces fonctions sont déclenchées par le superviseur (Code interne) ou l'opérateur (IHM).*

### 1.1. Fonction : `assign(robot_id)`
* **Description** : Alloue une ressource robotique à une mission en attente.
* **Pré-condition** : La mission est au statut `PENDING`.
* **Actions** :
    1.  Associer l'identifiant `robot_id` à l'objet mission.
    2.  Verrouiller la mission pour empêcher une double assignation.
* **Transition d'état** : `PENDING` $\rightarrow$ `ASSIGNED`

### 1.2. Fonction : `start_approach()`
* **Description** : Lance le déplacement du robot vers le premier point de la mission (point de collecte).
* **Pré-condition** : La mission est au statut `ASSIGNED`.
* **Actions** :
    1.  Récupérer les coordonnées du **Waypoint 0**.
    2.  Instancier et envoyer un but (Goal) au client d'action ROS2 (`NavigateToPose`).
* **Transition d'état** : `ASSIGNED` $\rightarrow$ `APPROACHING`

### 1.3. Fonction : `revoke()`
* **Description** : Annule l'assignation du robot pendant la phase d'approche (robot vide).
* **Pré-condition** : La mission est au statut `APPROACHING`.
* **Actions** :
    1.  Envoyer une requête d'annulation (`CancelGoal`) au client d'action ROS2.
    2.  Dissocier l'identifiant `robot_id` de la mission.
* **Transition d'état** : `APPROACHING` $\rightarrow$ `PENDING`

### 1.4. Fonction : `proceed()`
* **Description** : Valide l'étape actuelle et envoie le robot vers la destination suivante.
* **Pré-condition** : La mission est au statut `WAITING`.
* **Actions** :
    1.  Incrémenter l'index du waypoint cible (`current_index + 1`).
    2.  Récupérer les coordonnées du nouveau Waypoint.
    3.  Envoyer un nouveau but (Goal) au client d'action ROS2.
* **Transition d'état** : `WAITING` $\rightarrow$ `DELIVERING`

### 1.5. Fonction : `suspend()`
* **Description** : Interrompt la mission pour rediriger le robot vers une zone de sécurité ou de dépose.
* **Pré-condition** : La mission est au statut `WAITING` ou `DELIVERING`.
* **Actions (Cas A - Depuis WAITING)** :
    1.  Calculer le point de dépose sécurisé le plus proche.
    2.  Envoyer ce but au client d'action ROS2.
* **Actions (Cas B - Depuis DELIVERING)** :
    1.  **Ne pas annuler** l'action ROS en cours (le robot finit son segment).
    2.  Mettre à jour le statut interne immédiatement pour traiter l'arrivée comme une suspension.
* **Transition d'état** : `WAITING` / `DELIVERING` $\rightarrow$ `SUSPENDING`

### 1.6. Fonction : `start_discharge()`
* **Description** : Initie la procédure de déchargement des marchandises une fois le robot sécurisé.
* **Pré-condition** : La mission est au statut `SUSPENDING` (et le robot est arrivé).
* **Actions** :
    1.  Déclencher le mécanisme de déchargement ou notifier l'opérateur pour action manuelle.
* **Transition d'état** : `SUSPENDING` $\rightarrow$ `DISCHARGING`

### 1.7. Fonction : `release()`
* **Description** : Libère le robot une fois le déchargement validé.
* **Pré-condition** : La mission est au statut `DISCHARGING`.
* **Actions** :
    1.  Dissocier l'identifiant `robot_id`.
    2.  Sauvegarder l'index du dernier waypoint atteint (pour reprise future).
* **Transition d'état** : `DISCHARGING` $\rightarrow$ `PENDING`

### 1.8. Fonction : `complete()`
* **Description** : Valide la fin de la mission après la dernière livraison.
* **Pré-condition** : La mission est au statut `WAITING` (et `current_index` == dernier point).
* **Actions** :
    1.  Archiver la mission dans l'historique.
    2.  Notifier l'IHM du succès.
* **Transition d'état** : `WAITING` $\rightarrow$ `FINISHED`

---

## 2. CALLBACKS (RÉACTIONS SYSTÈME)
*Ces fonctions sont déclenchées automatiquement par les retours du serveur d'action ROS2 (`NavigateToPose`).*

### 2.1. Fonction : `on_arrival_cb()`
* **Déclencheur** : Réception du résultat ROS `SUCCEEDED`.
* **Logique décisionnelle** :
    * **A. Si État précédent == `APPROACHING`** :
        * Le robot est arrivé au point de collecte.
        * Transition : $\rightarrow$ `WAITING`
    * **B. Si État précédent == `DELIVERING`** :
        * Le robot est arrivé à un point intermédiaire ou final.
        * Transition : $\rightarrow$ `WAITING`
    * **C. Si État précédent == `SUSPENDING`** :
        * Le robot a atteint le point de dépose d'urgence (ou fini son trajet suspendu).
        * Transition : $\rightarrow$ `DISCHARGING` *(Ajustement : Transition directe selon diagramme)*

### 2.2. Fonction : `on_failure_cb()`
* **Déclencheur** : Réception du résultat ROS `ABORTED` ou `CANCELED` (hors appel revoke).
* **Pré-condition** : Tout état actif impliquant un mouvement (`APPROACHING`, `DELIVERING`, `SUSPENDING`).
* **Actions** :
    1.  Logger le code d'erreur ROS.
    2.  Ne pas libérer automatiquement le robot (pour diagnostic).
* **Transition d'état** : **TOUT ÉTAT ACTIF** $\rightarrow$ `FAILED`