import math
from geometry_msgs.msg import Pose
from .base_strategy import AllocatorStrategy
from .allocation_interface import AllocationDecision, AllocationAction
from mission_manager.core.mission import MissionState

class ClosestStrategy(AllocatorStrategy):
    def allocate(self) -> list[AllocationDecision]:
        decisions = []
        
        # --- 1. Récupération des données ---
        all_missions = self.registry.get_missions_list()
        all_robots_dict = self.robot_pool.get_all_robots()
        
        # --- 2. Identification des robots occupés ---
        busy_robot_ids = set()
        for m in all_missions:
            # Si un robot est assigné et que la mission n'est pas finie/échouée
            if m.assigned_robot_id and m.state not in [MissionState.FINISHED, MissionState.FAILED]:
                busy_robot_ids.add(m.assigned_robot_id)
        
        # --- 3. Liste des robots disponibles ---
        available_robots = []
        for r_id, robot in all_robots_dict.items():
            if r_id not in busy_robot_ids:
                available_robots.append(robot)
        
        # --- 4. Récupération des missions PENDING ---
        pending_missions = self.registry.get_missions_by_state(MissionState.PENDING)
        
        # --- 5. Tri des missions : Priorité > Date ---
        pending_missions.sort(
            key=lambda m: (
                -m.priority, 
                m.request_stamp.sec, 
                m.request_stamp.nanosec
            )
        )
        
        # --- 6. Allocation : Le plus proche d'abord ---
        for mission in pending_missions:
            if not available_robots:
                break # Plus de robots libres, on arrête
            
            if not mission.waypoints:
                self.node.get_logger().warn(f"Mission {mission.mission_id} has no waypoints. Skipping.")
                continue

            # La cible est le premier waypoint de la mission
            target_pose = mission.waypoints[0]
            
            best_robot = None
            min_dist = float('inf')
            
            # On cherche le robot le plus proche parmi ceux disponibles
            for robot in available_robots:
                dist = self._compute_distance(robot.get_position(), target_pose)
                
                if dist < min_dist:
                    min_dist = dist
                    best_robot = robot
            
            # Si on a trouvé un candidat (ce qui devrait être le cas s'il y a des robots dispos)
            if best_robot:
                decision = AllocationDecision(
                    action=AllocationAction.ASSIGN_AND_START,
                    mission_id=mission.mission_id,
                    robot_id=best_robot.robot_id,
                    reason=f"Closest available (dist={min_dist:.2f}m)"
                )
                decisions.append(decision)
                
                # IMPORTANT : On retire ce robot de la liste des dispos pour ne pas lui assigner 2 missions
                available_robots.remove(best_robot)
            
        return decisions

    def _compute_distance(self, pose1: Pose, pose2: Pose) -> float:
        """Calcule la distance euclidienne 2D entre deux Poses ROS"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.hypot(dx, dy)