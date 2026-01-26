# utility_strategy.py

import time
from .base_strategy import AllocatorStrategy
from .allocation_interface import AllocationDecision, AllocationAction
from mission_manager.core.mission import Mission, MissionState
from mission_manager.config import AVERAGE_WAITING_TIME, REQUEST_PATH_COMPUTING_TIMEOUT, ROBOT_QUALITY_S, ROBOT_AVERAGE_SPEED, COMPUTE_PLAN_TIMEOUT
from mission_manager.core.robot import Robot

class UtilityStrategy(AllocatorStrategy):

    def allocate(self) -> list[AllocationDecision]:
        decisions = []
        
        all_missions = self.registry.get_missions_list()
        all_robots = list(self.robot_pool.get_all_robots().values())
        
        nb_missions = len(all_missions)
        nb_robots = len(all_robots)
        
        self.logger.info(f"-robots[{nb_robots}]: {[r.robot_id for r in all_robots]}")
        self.logger.info(f"-missions[{nb_missions}]: {[m.mission_id for m in all_missions]}")

        if nb_missions == 0 or nb_robots == 0:
            return []
        
        # --- 1. Remplir la matrice de coûts ---
        cost_matrix = np.zeros((nb_missions, nb_robots))
        
        for i_m, mission in enumerate(all_missions):
            for i_r, robot in enumerate(all_robots):
                cost = self.compute_cost(mission.mission_id, robot.robot_id)
                cost_matrix[i_m, i_r] = cost

        # --- 2. Calcul Utilité ---
        quality_matrix = np.full((nb_missions, nb_robots), ROBOT_QUALITY_S)
        utility_matrix = quality_matrix - cost_matrix
        utility_matrix[utility_matrix < 0] = 0
        
        self.node.get_logger().info(f"Utility Matrix:\n{utility_matrix}")
        
        # --- 3. Hongrois ---
        assignments, _ = solve_hungarian_max(utility_matrix)
        
        # --- 4. Génération des Décisions ---
        for i_m, i_r in assignments:
            mission = all_missions[i_m] # Mission Cible
            robot = all_robots[i_r]
            
            utility_score = utility_matrix[i_m, i_r]
            
            # A. Filtrage basique
            if utility_score <= 0:
                continue
            
            if mission.assigned_robot_id == robot.robot_id:
                continue # Déjà assigné, on ne fait rien
            
            # B. Analyse de l'état actuel du robot
            current_mission = self.registry.get_mission_by_robot(robot.robot_id)
            
            action = None
            reason = f"U={utility_score:.1f}"

            # --- C. Application des Règles Strictes ---
            
            if current_mission is None or current_mission.state in [MissionState.FINISHED, MissionState.FAILED]:
                # Cas : Robot Libre
                action = AllocationAction.ASSIGN_AND_START
                
            else:
                # Cas : Robot Occupé -> On tente de remplacer
                
                # Règle 1 : REVOKE uniquement depuis APPROACHING
                if current_mission.state == MissionState.APPROACHING:
                    action = AllocationAction.REVOKE
                    
                # Règle 2 : SUSPEND uniquement depuis WAITING ou DELIVERING
                elif current_mission.state in [MissionState.WAITING, MissionState.DELIVERING]:
                    action = AllocationAction.SUSPEND
                    
                # Règle 3 : PENDING / ASSIGNED -> Erreur (Trop tôt pour préempter ou race condition)
                elif current_mission.state in [MissionState.PENDING, MissionState.ASSIGNED]:
                    self.node.get_logger().error(
                        f"Algo tried to preempt robot {robot.robot_id} in state {current_mission.state}. Action ignored."
                    )
                    action = None # On ne fait rien
                    
                # Règle 4 : Autres états (SUSPENDING, DISCHARGING) -> Nothing
                else:
                    # On considère que c'est déjà en cours de gestion, on ne touche pas
                    action = None

            # D. Ajout si une action valide a été trouvée
            if action is not None:
                decision = AllocationDecision(
                    action=action,
                    mission_id=mission.mission_id,
                    robot_id=robot.robot_id,
                    reason=reason
                )
                decisions.append(decision)
            
        return decisions
    
    
    def compute_cost(self, mission_id, robot_id):
        
        missions_registry = self.registry
        robots_pool = self.robot_pool
        
        robot : Robot = robots_pool.get_robot(robot_id)
        mission : Mission = missions_registry.get_mission(mission_id)
        
        mission_current = missions_registry.get_mission_by_robot(robot_id)

        
        pose_approch_start = None
        
        navigation_cost = 0
        waiting_cost = 0
        approaching_cost = 0
        aborting_mission_cost = 0
        
        start_mission_waypoint_idx = mission.goal_waypoint_idx
        
        # Cas 1 : La robot est déja assigné à la mission (=> Cour nul)
        if mission_current is not None and mission_current.assigned_robot_id == robot_id:
            return 0
        
        if start_mission_waypoint_idx is None or start_mission_waypoint_idx < 0:
            start_mission_waypoint_idx = 0
        
        # Cas 2 : Le robot est sur une autre mission en cours
        if mission_current is not None and mission_current.state in [MissionState.DELIVERING, MissionState.SUSPENDING]:
            # Sécurisation de l'accès à la liste waypoints
            idx = mission_current.goal_waypoint_idx if mission_current.goal_waypoint_idx is not None else 0
            
            if mission_current.waypoints and idx < len(mission_current.waypoints):
                pose_approch_start = mission_current.waypoints[idx]
            else:
                # Fallback si l'index est pourri
                pose_approch_start = robot.get_position()

            if robot.adapter.estimated_time_remaining:
                navigation_cost += robot.adapter.estimated_time_remaining
        else:
            pose_approch_start = robot.get_position()
          
        if not mission.waypoints:
             return float('inf')
         
        goal_handle_future = robot.adapter.make_plan_async(
            pose_approch_start, 
            mission.waypoints[start_mission_waypoint_idx]
        )
        if goal_handle_future is None:
            self.node.get_logger().error(f"Failed to call make_plan for robot {robot_id}")
            return float('inf')
        
        if mission_current is not None and mission_current.state in [MissionState.WAITING, MissionState.DISCHARGING, MissionState.DELIVERING, MissionState.SUSPENDING]:
            # En cours ou en fin de mission
            waiting_cost += AVERAGE_WAITING_TIME
            if mission.priority != 2: 
                # Mission non urgente
                aborting_mission_cost += ROBOT_QUALITY_S

        start_time = time.time()
        while not goal_handle_future.done():
            if time.time() - start_time > REQUEST_PATH_COMPUTING_TIMEOUT:
                self.node.get_logger().error(f"Timeout waiting for planner server response (Robot {robot_id})")
                return float('inf')
            time.sleep(0.01)
            
        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"Planning request rejected by robot {robot_id}")
            return float('inf')
        
        result_future = goal_handle.get_result_async()

        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > COMPUTE_PLAN_TIMEOUT:
                self.node.get_logger().error(f"Timeout waiting for planning result (Robot {robot_id})")
                return float('inf')
            time.sleep(0.01) # Pause critique

        
        wrapped_result = result_future.result()
        path = wrapped_result.result.path
        
        path_length = self._calculate_path_length(path)
        approaching_cost = path_length / ROBOT_AVERAGE_SPEED
        
        if mission_current is None:
            return approaching_cost
        match mission_current.state:
            case MissionState.ASSIGNED:
                return approaching_cost
            case MissionState.APPROACHING:
                return approaching_cost
            case MissionState.WAITING:
                return aborting_mission_cost + waiting_cost + approaching_cost
            case MissionState.DELIVERING:
                return aborting_mission_cost + navigation_cost + waiting_cost + approaching_cost
            case MissionState.SUSPENDING:
                return navigation_cost + waiting_cost + approaching_cost
            case MissionState.DISCHARGING:
                return waiting_cost + approaching_cost
            case MissionState.FAILED:
                return approaching_cost
            case MissionState.FINISHED:
                return approaching_cost
            case _:
                self.node.get_logger().error("Unexpected robot.state during cost computation")
                return float('inf')
        
    def _calculate_path_length(self, path_msg):
        """Calcule la distance cumulée d'un nav_msgs/Path"""
        import math
        length = 0.0
        if not path_msg or not path_msg.poses:
            return 0.0
            
        for i in range(len(path_msg.poses) - 1):
            p1 = path_msg.poses[i].pose.position
            p2 = path_msg.poses[i+1].pose.position
            dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
            length += dist
        return length
    
import numpy as np
from scipy.optimize import linear_sum_assignment

def solve_hungarian_max(matrix):
    """
    Résout le problème d'affectation pour maximiser la somme.
    Retourne une liste de tuples (index_ligne, index_colonne).
    """
    # 1. Vérification de sécurité compatible NumPy
    if matrix is None:
        return [], 0
        
    # Si c'est déjà un array numpy
    if isinstance(matrix, np.ndarray):
        if matrix.size == 0:
            return [], 0
    # Si c'est une liste standard
    elif len(matrix) == 0:
        return [], 0
        
    M = np.array(matrix)
    
    # 2. Résolution
    # linear_sum_assignment minimise par défaut. On passe -M pour maximiser.
    row_ind, col_ind = linear_sum_assignment(M, maximize=True)
    
    assignments = []
    total_value = 0
    
    for r, c in zip(row_ind, col_ind):
        value = M[r, c]
        assignments.append((r, c))
        total_value += value
        
    return assignments, total_value