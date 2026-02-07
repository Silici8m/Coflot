# mission_manager_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from fleet_interfaces.msg import MissionRequest, RobotStateArray, MissionState, MissionStateArray
from std_msgs.msg import String

from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry

from mission_manager.allocation_strategies.allocation_interface import AllocationAction
from mission_manager.allocation_strategies.closest_strategy import ClosestStrategy
from mission_manager.allocation_strategies.naive_queue_strategy import NaiveQueueStrategy
from mission_manager.allocation_strategies.utility_strategy import UtilityStrategy

from mission_manager.config import FLEET_STATE_TOPIC, MISSION_REQUEST_TOPIC, VALIDATION_TOPIC, MISSIONS_STATE_TOPIC, ALLOCATION_INTERVAL, MISSION_PUBLICATION_INTERVAL


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.robot_pool = RobotPool(self)
        self.registry = MissionRegistry(self, self.robot_pool)
        
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Allocation strategy
        self.declare_parameter('allocation_strategy', 'utility')
        strategy_name = self.get_parameter('allocation_strategy').value
        
        self.get_logger().info(f"Strategy selected: {strategy_name}")
        
        if strategy_name == 'simple_queue':
            # On passe le pool et le registry ici !
            self.allocator = NaiveQueueStrategy(self, self.robot_pool, self.registry)
        
        elif strategy_name == 'utility':
            
            self.allocator = UtilityStrategy(self, self.robot_pool, self.registry)
        elif strategy_name == 'closest':
            
            self.allocator = ClosestStrategy(self, self.robot_pool, self.registry)
        else:
            raise ValueError(f"Unknown strategy: {strategy_name}")
        
        self.allocation_group = ReentrantCallbackGroup()
        
        
        # --- Publishers ---
        # Publisher pour l'état des missions
        self.missions_state_pub = self.create_publisher(
            MissionStateArray,
            MISSIONS_STATE_TOPIC,
            latching_qos
        )
        
        # Subscribers
        
        # Fleet state for robot pool updates
        self.create_subscription(
            RobotStateArray, 
            FLEET_STATE_TOPIC, 
            self.robot_pool.update_pool, 
            10
        )
        
        # Mission requests for new missions
        self.create_subscription(
            MissionRequest,
            MISSION_REQUEST_TOPIC,
            self.registry.add_mission,
            10
        )
        
        # Subscription for mission validation
        self.create_subscription(
            String,
            VALIDATION_TOPIC,
            self._validation_cb,
            10
        )

        # Timers
        # Mission allocation
        self.create_timer(
            ALLOCATION_INTERVAL, 
            self.allocation_loop, 
            callback_group=self.allocation_group
        ) 
        # mission state publication
        self.create_timer(
            MISSION_PUBLICATION_INTERVAL,
            self.publish_missions_state
        )
        
        self.get_logger().info("Mission Manager started.")
        
    def _validation_cb(self, msg):
        # msg.data contient l'ID de la mission validée
        self.registry.handle_validation(msg.data)
        
    def publish_missions_state(self):
        msg = MissionStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # On récupère toutes les missions (MissionRegistry doit avoir get_missions())
        # Si get_missions() retourne un dict, utilisez .values()
        all_missions = self.registry.get_missions_list() 

        for m in all_missions:
            s = MissionState()
            s.mission_id = m.mission_id
            s.priority = m.priority
            s.waypoints = m.waypoints
            
            # Gestion des champs optionnels / conversion types
            s.assigned_robot_id = m.assigned_robot_id if m.assigned_robot_id else ""
            s.status = m.state.name # Conversion Enum -> String (PENDING, etc.)
            s.goal_waypoint_idx = m.goal_waypoint_idx if m.goal_waypoint_idx is not None else -1
            s.request_stamp = m.request_stamp
            
            msg.missions.append(s)
            
        self.missions_state_pub.publish(msg)
        
    def allocation_loop(self):
        """
        Boucle principale : Demande des décisions -> Exécute.
        """
        try:
            # 1. Obtenir les décisions de la stratégie
            self._logger.info("\nAllocation loop")
            decisions = self.allocator.allocate()
        except Exception as e:
            self.get_logger().error(f"Allocation error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        if not decisions:
            # self._logger.info("Aucune décision")
            return
        #self._logger.info(f"{len(decisions)} décisions :")

        # 2. Exécution des décisions
        for d in decisions:
            self.get_logger().info(f"[ALLOC] Action {d.action.name}: Robot {d.robot_id} (Reason: {d.reason})")
            
            try:
                # --- ROUTAGE DES ACTIONS ---
                if d.action == AllocationAction.ASSIGN_AND_START:
                    # Ici on a besoin de l'ID de la nouvelle mission
                    self._execute_start(d.mission_id, d.robot_id)
                    
                elif d.action == AllocationAction.REVOKE:
                    # On n'a pas besoin de d.mission_id ici, on annule juste ce que le robot fait
                    self._execute_revoke(d.robot_id)
                    
                elif d.action == AllocationAction.SUSPEND:
                    # On suspend juste ce que le robot fait
                    self._execute_suspend(d.robot_id)
                    
                else:
                    self.get_logger().warn(f"Action inconnue ou non gérée: {d.action}")
            
            except Exception as e:
                self.get_logger().error(f"Execution failed for decision {d}: {e}")
            self._logger.info("Allocation loop finished\n")

    # --- Primitives d'Exécution ---

    def _execute_start(self, mission_id, robot_id):
        """
        Assigne une mission à un robot et lance l'approche.
        """
        # 1. Tentative d'assignation dans le registre (Lien Logique)
        if self.registry.mission_assign(mission_id, robot_id):
            # 2. Si succès, déclenchement du mouvement (Commande Physique)
            self.registry.mission_start_approaching(mission_id)
        else:
            self.get_logger().error(f"Failed to assign mission {mission_id} to {robot_id}")

    def _execute_revoke(self, robot_id):
        """
        Annule UNIQUEMENT la mission courante du robot.
        La nouvelle mission sera assignée au prochain cycle (via ASSIGN_AND_START).
        """
        current_mission = self.registry.get_mission_by_robot(robot_id)
        
        if current_mission:
            self.get_logger().warn(f"REVOKING mission {current_mission.mission_id} on robot {robot_id}")
            # Appel au registre pour nettoyer l'ancienne mission (State -> FAILED/CANCELLED)
            self.registry.mission_revoke(current_mission.mission_id)
        else:
            self.get_logger().warn(f"Received REVOKE for robot {robot_id} but no mission found.")

    def _execute_suspend(self, robot_id):
        """
        Suspend UNIQUEMENT la mission courante du robot.
        La nouvelle mission sera assignée au prochain cycle (via ASSIGN_AND_START).
        """
        current_mission = self.registry.get_mission_by_robot(robot_id)
        
        if current_mission:
            self.get_logger().warn(f"SUSPENDING mission {current_mission.mission_id} on robot {robot_id}")
            # Appel au registre pour passer l'ancienne en SUSPENDING
            self.registry.mission_suspend(current_mission.mission_id)
        else:
            self.get_logger().warn(f"Received SUSPEND for robot {robot_id} but no mission found.")
            
            
def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    executor = MultiThreadedExecutor() # Important pour éviter le deadlock !
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()