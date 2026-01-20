// MissionManager.js

class MissionManager {
    constructor() {
        this.missions = [];
    }

    /**
     * Met à jour la liste locale des missions
     * @param {Array} missionArrayMsg - Le champ .missions du message ROS
     */
    update(missionArrayMsg) {
        this.missions = missionArrayMsg;
        this.render();
    }

    /**
     * Génère l'affichage HTML dans le panel de suivi
     */
    render() {
        const container = document.getElementById('missionDisplay');
        if (!container) return;

        if (this.missions.length === 0) {
            container.innerHTML = "<em>Aucune mission en cours</em>";
            return;
        }

        let html = "";
        this.missions.forEach(m => {
            const statusClass = this.getStatusClass(m.status);
            const priorityLabel = this.getPriorityLabel(m.priority);
            
            html += `
                <div class="mission-card ${statusClass}">
                    <div class="mission-header">
                        <span class="mission-id">${m.mission_id}</span>
                        <span class="mission-priority">${priorityLabel}</span>
                    </div>
                    <div class="mission-body">
                        <p><strong>Statut:</strong> ${m.status}</p>
                        <p><strong>Robot:</strong> ${m.assigned_robot_id || "<em>En attente...</em>"}</p>
                        <div class="mission-progress">
                            WP: ${m.goal_waypoint_idx + 1} / ${m.waypoints.length}
                        </div>
                    </div>
                </div>
            `;
        });
        container.innerHTML = html;
    }

    getStatusClass(status) {
        if (status === 'FINISHED') return 'mission-success';
        if (status === 'FAILED') return 'mission-error';
        if (status === 'PENDING') return 'mission-warning';
        return 'mission-active';
    }

    getPriorityLabel(prio) {
        const labels = { 0: "Normal", 1: "Prioritaire", 2: "Urgente" };
        return labels[prio] || "Inconnu";
    }
}