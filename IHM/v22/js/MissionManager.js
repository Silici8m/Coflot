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
            container.innerHTML = "<div style='padding:15px; color:#666; font-style:italic; text-align:center;'>Aucune mission en cours</div>";
            return;
        }

        let html = "";
        this.missions.forEach(m => {
            const statusClass = this.getStatusClass(m.status);
            const priorityInfo = this.getPriorityInfo(m.priority);
            const btnState = this.getValidationButtonState(m);
            
            // Bouton
            let buttonHtml = '';
            if (btnState.clickable) {
                buttonHtml = `<button class="btn-validation ${btnState.cssClasses}" onclick="validateMission('${m.mission_id}')">✔</button>`;
            } else {
                buttonHtml = `<button class="btn-validation ${btnState.cssClasses}" disabled>✔</button>`;
            }

            // Progression
            const progress = `WP: ${m.goal_waypoint_idx >=0 ? m.goal_waypoint_idx : 0} / ${m.waypoints.length}`;

            // Structure HTML Compacte
            html += `
                <div class="mission-card-compact ${statusClass}">
                    <div class="mc-left">
                        <div class="mc-id" title="${m.mission_id}">${m.mission_id}</div>
                        <div class="mc-details">
                            <span class="prio-dot ${priorityInfo.cssClass}" title="Priorité"></span>
                            <span class="mc-status">${m.status}</span>
                            <span class="mc-robot">(${m.assigned_robot_id || "?"})</span>
                            <span style="font-size:0.7rem; color:#666; margin-left:auto;">${progress}</span>
                        </div>
                    </div>
                    <div class="mc-right">
                        ${buttonHtml}
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
        if (['ASSIGNED', 'APPROACHING', 'DELIVERING', 'DISCHARGING', 'SUSPENDING', 'WAITING'].includes(status)) return 'mission-active';
        return '';
    }

    getPriorityInfo(prio) {
        // Retourne un objet avec le Label et la classe CSS
        switch(prio) {
            case 2: return { label: "High", cssClass: "tag-high" };
            case 3: return { label: "URGENT", cssClass: "tag-urgent" };
            default: return { label: "Normal", cssClass: "tag-normal" };
        }
    }

    getValidationButtonState(m) {
        let classes = [];
        let isClickable = false;

        // Base
        if (m.status === 'WAITING') {
            classes.push('btn-val-waiting'); isClickable = true;
        } else if (m.status === 'DISCHARGING') {
            classes.push('btn-val-discharging'); isClickable = true;
        } else {
            classes.push('btn-val-disabled'); isClickable = false;
        }

        // Spécial (Surcharge)
        const isSuspending = (m.status === 'SUSPENDING');
        const isInterruptedDischarge = (m.status === 'DISCHARGING' && m.goal_waypoint_idx < m.waypoints.length);

        if (isSuspending || isInterruptedDischarge) {
            classes.push('btn-val-special');
            isClickable = true;
        }

        return { cssClasses: classes.join(' '), clickable: isClickable };
    }
}