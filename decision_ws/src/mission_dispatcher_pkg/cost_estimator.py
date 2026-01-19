# cost_estimator.py

import rclpy
import math
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class CostEstimator:
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        # Client pour demander le chemin au Planner Server
        # Attention : topic par défaut nav2, à adapter si namespace
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        
        self.ROBOT_SPEED = 0.5 # m/s (À paramétrer)

    def compute_cost(self, robot, mission):
        """
        Calcule le temps estimé (secondes) pour qu'un robot commence une mission.
        Gère le cas où le robot est déjà occupé.
        """
        
        # 1. Déterminer le point de départ du calcul
        start_pose = None
        base_time_penalty = 0.0
        
        if robot.adapter.is_navigating():
            start_pose = robot.adapter.current_goal_pose
            
            # On ajoute le temps qu'il lui reste à tirer
            # (Note: estimated_time_remaining vient du feedback Nav2)
            if robot.adapter.estimated_time_remaining:
                base_time_penalty = robot.adapter.estimated_time_remaining
        else:
            # Le robot est libre.
            # Départ = Là où il est maintenant (via RobotState)
            start_pose = robot.get_position() # geometry_msgs/Pose

        if start_pose is None:
            return float('inf') # Impossible de calculer

        # 2. Convertir Pose en PoseStamped (requis par Nav2)
        start_stamped = self._to_pose_stamped(start_pose)
        
        # La destination est le premier waypoint de la mission candidate
        goal_stamped = self._to_pose_stamped(mission.waypoints[0]) 

        # 3. Calculer la longueur du chemin (Appel Service Nav2)
        path_length = self._get_path_length(start_stamped, goal_stamped)
        
        if path_length is None:
            return float('inf') # Pas de chemin trouvé (mur, hors map)

        # 4. Conversion en temps
        travel_time = path_length / self.ROBOT_SPEED
        
        total_cost = base_time_penalty + travel_time
        return total_cost

    def _get_path_length(self, start, goal):
        """Appelle l'action ComputePathToPose et somme les distances."""
        
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.logger.warn("ComputePathToPose server not available")
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = "GridBased" # Ou laisser vide pour défaut
        
        # Appel Synchrone (bloquant court) pour simplifier l'algo d'allocation
        future = self.client.send_goal_async(goal_msg)
        
        # Astuce : On attend le résultat immédiatement
        # Attention : Dans un vrai système temps réel strict, on ferait autrement,
        # mais pour une boucle d'alloc 1Hz c'est OK.
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        if not future.done():
            return None # Timeout
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            return None

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future, timeout_sec=1.0)
        
        result = res_future.result().result
        
        # Calcul de la distance géométrique le long du chemin
        if not result or not result.path.poses:
            return None
            
        return self._calculate_geometric_distance(result.path.poses)

    def _calculate_geometric_distance(self, poses):
        length = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            dist = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
            length += dist
        return length

    def _to_pose_stamped(self, pose_msg):
        # Helper pour wrapper une Pose en PoseStamped
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.node.get_clock().now().to_msg()
        # Si c'est déjà une PoseStamped, on prend .pose, sinon on prend l'objet tel quel
        if hasattr(pose_msg, 'pose'):
             ps.pose = pose_msg.pose
        else:
             ps.pose = pose_msg
        return ps