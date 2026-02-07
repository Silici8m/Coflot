# mission_manager_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from fleet_interfaces.msg import MissionRequest, RobotStateArray, MissionState as MissionStateMsg, MissionStateArray as MissionStateArrayMsg
from std_msgs.msg import String

from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry
from mission_manager.core.mission import MissionState

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
            MissionStateArrayMsg,
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
        msg = MissionStateArrayMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # On récupère toutes les missions (MissionRegistry doit avoir get_missions())
        # Si get_missions() retourne un dict, utilisez .values()
        all_missions = self.registry.get_missions_list() 

        for m in all_missions:
            s = MissionStateMsg()
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
            decisions = self.allocator.allocate()
        except Exception as e:
            self.get_logger().error(f"Allocation error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return

        if not decisions:
            return
        
        self._logger.info(f"/!\ {len(decisions)} DECISIONS :")

        # 2. Exécution des décisions
        for d in decisions:
            self.get_logger().info(f" - {d.action.name} {d.robot_id} | {d.mission_id} ({d.reason})")
            
            try:
                target_mission = self.registry.get_mission(d.mission_id)
                current_mission = self.registry.get_mission_by_robot(d.robot_id)

                # --- ROUTAGE DES ACTIONS ---

                # CAS 1 : Démarrage standard (ASSIGN_AND_START)
                if d.action == AllocationAction.ASSIGN_AND_START:
                    # Sanity Check : On ne démarre que si la mission cible est bien PENDING
                    # (Si elle est déjà ASSIGNED à qqun d'autre par erreur, on stop)
                    if target_mission.state == MissionState.PENDING:
                        self._execute_start(d.mission_id, d.robot_id)
                    else:
                        self.get_logger().warn(f"Ignored START: Target {d.mission_id} is {target_mission.state.name}")

                # CAS 2 : Annulation (REVOKE)
                elif d.action == AllocationAction.REVOKE:
                    # Sanity Check : On ne révoque que si on est toujours en approche
                    if current_mission and current_mission.state == MissionState.APPROACHING:
                        self._execute_revoke(d.robot_id)
                    else:
                        self.get_logger().warn(f"Ignored REVOKE: Robot {d.robot_id} state changed (expected APPROACHING)")
                
                # CAS 3 : Suspension (SUSPEND)
                elif d.action == AllocationAction.SUSPEND:
                    # Sanity Check : On ne suspend que si on travaille
                    if current_mission and current_mission.state in [MissionState.WAITING, MissionState.DELIVERING]:
                        self._execute_suspend(d.robot_id)
                    else:
                        self.get_logger().warn(f"Ignored SUSPEND: Robot {d.robot_id} state changed")
                
                # CAS 4 : NOTHING (Ne devrait pas arriver ici si filtré par la stratégie, mais au cas où)
                elif d.action == AllocationAction.NOTHING:
                    pass 
                                        
                else:
                    # Fallback pour debug
                    target_state = target_mission.state.name if target_mission else "None"
                    current_state = current_mission.state.name if current_mission else "None"
                    self.get_logger().warn(
                        f"[Refus Allocation] Action {d.action.name} impossible / non gérée. "
                        f"Target: {target_state} | Current: {current_state}"
                    )

            except Exception as e:
                self.get_logger().error(f"Execution failed for decision {d}: {e}")

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