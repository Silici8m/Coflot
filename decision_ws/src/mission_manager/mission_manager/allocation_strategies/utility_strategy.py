# utility_strategy.py

import time
import math
from typing import List, Tuple, Any, Optional, Union

import numpy as np
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

from .base_strategy import AllocatorStrategy
from .allocation_interface import AllocationDecision, AllocationAction

from mission_manager.core.mission import MissionPriority, MissionState, Mission
from mission_manager.config import (
    AVERAGE_WAITING_TIME, 
    REQUEST_PATH_COMPUTING_TIMEOUT, 
    ROBOT_QUALITY_S, 
    ROBOT_AVERAGE_SPEED, 
    COMPUTE_PLAN_TIMEOUT, 
    COEF_MISSION_PRIORITAIRE, 
    COEF_MISSION_URGENTE,
    STEAL_COST
)


def solve_hungarian_max(matrix: Union[List[List[float]], np.ndarray]) -> Tuple[List[Tuple[int, int]], float]:
    """
    Solves the linear assignment problem to maximize the total utility.

    This function wraps scipy's `linear_sum_assignment` (which minimizes cost)
    by passing the negative of the matrix to achieve maximization.

    Args:
        matrix (Union[List[List[float]], np.ndarray]): A 2D cost/utility matrix.

    Returns:
        Tuple[List[Tuple[int, int]], float]: A tuple containing:
            - A list of (row_index, col_index) tuples representing the optimal assignment.
            - The total sum of the values of the assigned elements.
    """
    # 1. Vérification de sécurité compatible NumPy
    if matrix is None:
        return [], 0.0
        
    # Si c'est déjà un array numpy
    if isinstance(matrix, np.ndarray):
        if matrix.size == 0:
            return [], 0.0
    # Si c'est une liste standard
    elif len(matrix) == 0:
        return [], 0.0
        
    M = np.array(matrix)
    
    # 2. Résolution
    # linear_sum_assignment minimise par défaut. On passe -M pour maximiser.
    row_ind, col_ind = linear_sum_assignment(M, maximize=True)
    
    assignments = []
    total_value = 0.0
    
    for r, c in zip(row_ind, col_ind):
        value = M[r, c]
        assignments.append((r, c))
        total_value += value
        
    return assignments, total_value

class UtilityStrategy(AllocatorStrategy):
    """
    Implements a global optimization strategy based on Utility.

    This strategy computes a Utility matrix (Quality - Cost) for every robot-mission pair
    and solves the assignment problem using the Hungarian algorithm (Munkres) to maximize
    the global utility of the fleet. It accounts for mission priorities, travel times,
    and transition costs (revocation/suspension penalties).
    """

    def allocate(self) -> List[AllocationDecision]:
        """
        Executes the utility-based allocation logic.

        The process follows these steps:
        1. Construct a Cost Matrix based on temporal costs (travel time + penalties).
        2. Derive a Utility Matrix (Quality - Cost), adjusted by mission Priority coefficients.
        3. Solve the assignment problem using the Hungarian algorithm to maximize global utility.
        4. Generate allocation decisions for the optimal assignments found.

        Returns:
            List[AllocationDecision]: A list of decisions to be executed by the dispatcher.
        """
        decisions: List[AllocationDecision] = []
        
        all_missions_full = self.registry.get_missions_list()
        all_missions: List[Mission] = [m for m in all_missions_full if m.state not in [MissionState.FINISHED, MissionState.FAILED]]
        all_robots = list(self.robot_pool.get_all_robots().values())
        
        nb_missions = len(all_missions)
        nb_robots = len(all_robots)

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
        for i_m in range(nb_missions):
            match all_missions[i_m].priority:
                case MissionPriority.PRIORITAIRE.value:
                    utility_matrix[i_m, :] *= COEF_MISSION_PRIORITAIRE
                case MissionPriority.URGENTE.value:
                    utility_matrix[i_m, :] *= COEF_MISSION_URGENTE
                case _:
                    pass
        
        # --- 3. Hongrois ---
        assignments, _ = solve_hungarian_max(utility_matrix)
        
        # --- 4. Génération des Décisions ---
        for i_m, i_r in assignments:
            mission = all_missions[i_m] # Mission Cible
            robot = all_robots[i_r] # Robot Cible
            
            utility_score = utility_matrix[i_m, i_r]
            
            # Si l'utilité est nulle, on ne fait rien
            if utility_score <= 0:
                continue
            
            # Si la robot est déjà assignée à cette mission, on ne fait rien
            if mission.assigned_robot_id == robot.robot_id:
                continue
            
            # On récupère le mission actuelle du robot pour déterminer l'action à prendre
            current_mission = self.registry.get_mission_by_robot(robot.robot_id)
            required_action = self._get_required_action(current_mission)
            if required_action == AllocationAction.NOTHING:
                continue
            action = required_action
            reason = f"U={utility_score:.1f}"
            

            decisions.append(AllocationDecision(
                action=action,
                mission_id=mission.mission_id,
                robot_id=robot.robot_id,
                reason=reason
            ))
        return decisions
    
    
    def compute_cost(self, mission_id: str, robot_id: str) -> float:
        """
        Calculates the total temporal cost (in seconds) for a robot to perform a target mission.

        This method evaluates:
        1. Strict constraints (Anti-theft, Stability).
        2. Technical feasibility (Required actions like Suspend/Revoke).
        3. Transition penalties (Time to free the robot from its current task).
        4. Travel time to the target mission's first waypoint (using path planning).

        Args:
            mission_id (str): The unique identifier of the target mission.
            robot_id (str): The unique identifier of the candidate robot.

        Returns:
            float: The estimated cost in seconds. Returns float('inf') if the assignment is impossible.
        """
        missions_registry = self.registry
        robots_pool = self.robot_pool
        
        robot = robots_pool.get_robot(robot_id)
        target_mission = missions_registry.get_mission(mission_id)
        current_mission = missions_registry.get_mission_by_robot(robot_id)

        # ---------------------------------------------------------
        # 1. REGLES STRICTES
        # ---------------------------------------------------------

        steal_cost = 0.0

        # Règle : Anti-Vol. Interdit de prendre la mission d'un autre.
        if target_mission.assigned_robot_id is not None and target_mission.assigned_robot_id != robot_id:
            if target_mission.state == MissionState.APPROACHING and (current_mission is None or current_mission.priority <= target_mission.priority):
                steal_cost = STEAL_COST
            else:
                return float('inf')
            
        # Règle : Stabilité. Si c'est déjà ma mission, coût nul pour la garder.
        if target_mission.assigned_robot_id == robot_id:
            # Le coût est le temps restant pour finir ce que je fais
            if target_mission.state == MissionState.APPROACHING and robot.adapter.estimated_time_remaining is not None:
                return robot.adapter.estimated_time_remaining
            else:
                return 0.0

        # ---------------------------------------------------------
        # 2. REGLES DE L'ACTION REQUISE
        # ---------------------------------------------------------
        action = self._get_required_action(current_mission)

        if action == AllocationAction.SUSPEND:
            # Règle Métier : On ne suspend QUE pour une urgence
            if target_mission.priority != MissionPriority.URGENTE.value:
                return float('inf')
            
        if action == AllocationAction.NOTHING:
            # ON VEUT quand même calculer le coût (pour le réserver).
            if current_mission is not None and current_mission.state in [MissionState.SUSPENDING, MissionState.DISCHARGING]:
                pass # On laisse passer pour le calcul de temps
            else:
                return float('inf')

        # ---------------------------------------------------------
        # 3. CALCUL DU TEMPS DE LIBERATION (Pénalité de transition)
        # ---------------------------------------------------------
        
        time_to_free = 0.0
        # Par défaut, le trajet vers la cible part de la position actuelle
        start_pose_for_planning = robot.get_position() 


        # BRANCHE A : Démarrage direct (Robot Libre)
        if action == AllocationAction.ASSIGN_AND_START:
            time_to_free = 0.0
            start_pose_for_planning = robot.get_position()

        # BRANCHE B : Révocation (Interruption gratuite)
        elif action == AllocationAction.REVOKE:
            time_to_free = 0.0
            start_pose_for_planning = robot.get_position()

        # BRANCHE C : Suspension (Interruption coûteuse)
        elif action == AllocationAction.SUSPEND:
            # On doit finir le trajet + décharger
            path_rem = robot.adapter.estimated_time_remaining if robot.adapter.estimated_time_remaining else 0.0
            time_to_free = path_rem + AVERAGE_WAITING_TIME
            
            # Point de départ : Destination de la mission actuelle
            if current_mission.waypoints:
                idx = current_mission.goal_waypoint_idx if current_mission.goal_waypoint_idx is not None else -1
                start_pose_for_planning = current_mission.waypoints[idx]

        elif action == AllocationAction.NOTHING:
            # On calcule le temps qu'il reste à attendre
            if current_mission.state == MissionState.SUSPENDING:
                path_rem = robot.adapter.estimated_time_remaining or 0.0
                time_to_free = path_rem + AVERAGE_WAITING_TIME
                start_pose_for_planning = robot.get_position()
                
            elif current_mission.state == MissionState.DISCHARGING:
                time_to_free = AVERAGE_WAITING_TIME
                start_pose_for_planning = robot.get_position()
                
        
        if not target_mission.waypoints:
            return float('inf')

        target_wp_idx = target_mission.goal_waypoint_idx or 0
        target_pose = target_mission.waypoints[target_wp_idx]

        travel_time = self._get_travel_time(robot, start_pose_for_planning, target_pose)
        
        if travel_time is None:
            return float('inf')

        return time_to_free + travel_time + steal_cost


    def _get_travel_time(self, robot: Any, start_pose: Union[Pose, PoseStamped], target_pose: Union[Pose, PoseStamped]) -> Optional[float]:
        """
        Estimates the travel time between two points by querying the robot's plan server.

        This method sends an asynchronous path planning request to the robot's adapter
        and waits for the result (synchronously blocking with a timeout).

        Note:
            This method is blocking and includes `time.sleep` loops to wait for the
            async ROS 2 actions. This is necessary because the cost calculation
            must return a value to the Hungarian algorithm.

        Args:
            robot (Any): The robot instance (Robot object).
            start_pose (Union[Pose, PoseStamped]): The starting pose for the plan.
            target_pose (Union[Pose, PoseStamped]): The destination pose.

        Returns:
            Optional[float]: The estimated travel time in seconds, or None if planning failed.
        """
        try:
            # 1. Requête au serveur de planification
            goal_handle_future = robot.adapter.make_plan_async(start_pose, target_pose)
            
            if goal_handle_future is None:
                self.node.get_logger().error(f"Robot {robot.robot_id}: Planner not available")
                return None

            # 2. Attente de l'acceptation de la requête
            start_time = time.time()
            while not goal_handle_future.done():
                if time.time() - start_time > REQUEST_PATH_COMPUTING_TIMEOUT:
                    self.node.get_logger().error(f"Robot {robot.robot_id}: Timeout waiting for planner response")
                    return None
                time.sleep(0.005)
                
            goal_handle = goal_handle_future.result()
            if not goal_handle.accepted:
                return None # Requête rejetée par le robot
            
            # 3. Attente du calcul du chemin
            result_future = goal_handle.get_result_async()
            start_time = time.time() # Reset timer pour le calcul effectif
            while not result_future.done():
                if time.time() - start_time > COMPUTE_PLAN_TIMEOUT:
                    self.node.get_logger().error(f"Robot {robot.robot_id}: Timeout computing path")
                    return None
                time.sleep(0.005)

            # 4. Extraction et conversion en temps
            wrapped_result = result_future.result()
            path_msg = wrapped_result.result.path
            
            # Utilisation de la méthode de distance
            distance = self._calculate_path_length(path_msg)
            
            if distance == 0 and start_pose != target_pose:
                return float('inf') # Sécurité si path vide mais cibles différentes

            # t = d / v
            return distance / ROBOT_AVERAGE_SPEED
            
        except Exception as e:
            self.node.get_logger().error(f"Error in _get_travel_time: {e}")
            return None

    def _calculate_path_length(self, path_msg: Path) -> float:
        """
        Computes the total length (in meters) of a ROS 2 Path message.

        Sum the Euclidean distances between consecutive waypoints in the path.

        Args:
            path_msg (Path): The navigation path.

        Returns:
            float: The total length in meters.
        """
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