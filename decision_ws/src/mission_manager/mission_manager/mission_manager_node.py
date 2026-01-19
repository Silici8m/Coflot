# mission_manager_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from fleet_interfaces.msg import MissionRequest, RobotStateArray, MissionState, MissionStateArray
from std_msgs.msg import String

from mission_manager.core.robot_pool import RobotPool
from mission_manager.core.mission_registry import MissionRegistry

from mission_manager.allocation_strategies.allocation_interface import AllocationAction
from mission_manager.allocation_strategies.naive_queue_strategy import NaiveQueueStrategy

from mission_manager.config import FLEET_STATE_TOPIC, MISSION_REQUEST_TOPIC, VALIDATION_TOPIC, MISSIONS_STATE_TOPIC


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.robot_pool = RobotPool(self)
        self.registry = MissionRegistry(self, self.robot_pool)
        
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Allocation strategy
        self.declare_parameter('allocation_strategy', 'simple_queue')
        strategy_name = self.get_parameter('allocation_strategy').value
        
        self.get_logger().info(f"Strategy selected: {strategy_name}")
        
        if strategy_name == 'simple_queue':
            # On passe le pool et le registry ici !
            self.allocator = NaiveQueueStrategy(self.robot_pool, self.registry)
        
        elif strategy_name == 'smart_utility':
            # self.allocator = UtilityMatrixStrategy(self.robot_pool, self.registry)
            raise NotImplementedError("Smart strategy not enabled yet")
        else:
            raise ValueError(f"Unknown strategy: {strategy_name}")
        
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
        self.create_timer(1.0, self.allocation_loop) # Mission allocation
        self.create_timer(0.5, self.publish_missions_state) # mission state publishment
        
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
        # A. L'allocateur a déjà accès aux données, on l'appelle sans arguments
        try:
            decisions = self.allocator.allocate()
        except Exception as e:
            self.get_logger().error(f"Allocation error: {e}")
            return

        if not decisions:
            return

        # B. Exécution des décisions
        for d in decisions:
            self.get_logger().info(f"[ALLOC] Action {d.action.name}: Mission {d.mission_id} -> Robot {d.robot_id}")
            
            try:
                # ROUTAGE DES ACTIONS
                if d.action == AllocationAction.START:
                    self._execute_start(d.mission_id, d.robot_id)
                    
                elif d.action == AllocationAction.CANCEL_AND_START:
                    self._execute_cancel_and_start(d.mission_id, d.robot_id)
                
                # ... autres cas (SUSPEND, etc.)
                
            except Exception as e:
                self.get_logger().error(f"Execution failed for {d}: {e}")

    # --- Primitives d'Exécution ---

    def _execute_start(self, mission_id, robot_id):
        # Tente d'assigner. Si le robot est busy (race condition), ça renverra False.
        if self.registry.mission_assign(mission_id, robot_id):
            self.registry.mission_start_approaching(mission_id)

    def _execute_cancel_and_start(self, new_mission_id, robot_id):
        # 1. On révoque la mission actuelle
        current_mission = self.registry.get_mission_by_robot(robot_id)
        if current_mission:
            self.get_logger().warn(f"Preempting mission {current_mission.mission_id} on robot {robot_id}")
            self.registry.mission_revoke(current_mission.mission_id)
        
        # 2. On démarre la nouvelle
        self._execute_start(new_mission_id, robot_id)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()