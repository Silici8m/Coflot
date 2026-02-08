# fleet_simulation.py

import time
import math
import threading
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

# Messages
from fleet_interfaces.msg import RobotState, RobotStateArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Path

def euler_to_quaternion(yaw: float) -> Quaternion:
    """
    Converts a yaw angle (in radians) into a ROS 2 Quaternion message.

    Args:
        yaw (float): The angle in radians.

    Returns:
        Quaternion: The corresponding ROS geometry_msgs Quaternion.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class SimulatedRobot:
    """
    Represents the internal state of a single simulated robot.

    This class acts as a data container and state machine for the simulation,
    holding position, battery levels, and operational status.

    Attributes:
        robot_id (str): Unique identifier of the robot.
        x (float): Current X coordinate in the map frame.
        y (float): Current Y coordinate in the map frame.
        yaw (float): Current orientation in radians.
        battery (float): Current battery percentage (0-100).
        speed (float): Movement speed in m/s.
        status (str): Current operational status (e.g., "IDLE", "BUSY").
    """
    def __init__(self, robot_id: str, x: float, y: float, battery: float = 100.0, speed: float = 1.0) -> None:
        """
        Initializes a simulated robot.

        Args:
            robot_id (str): Unique identifier.
            x (float): Initial X position.
            y (float): Initial Y position.
            battery (float): Initial battery level (default: 100.0).
            speed (float): Movement speed in m/s (default: 1.0).
        """
        self.robot_id = robot_id
        self.x = float(x)
        self.y = float(y)
        self.yaw = 0.0 # Orientation en radians
        self.battery = float(battery)
        self.speed = speed # m/s
        self.status = "IDLE" 
        self.lock = threading.RLock()

    def get_pose(self) -> Pose:
        """
        Generates a standard ROS Pose message based on current internal state.

        Returns:
            Pose: The current position and orientation.
        """
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.orientation = euler_to_quaternion(self.yaw)
        return p

class MockFleetNode(Node):
    """
    Simulates a fleet of robots and their navigation stacks.

    This node replaces physical robots and the Nav2 stack for testing purposes.
    It provides Action Servers that mimic the `MapsToPose` and `ComputePathToPose`
    interfaces found in standard ROS 2 navigation stacks.

    Attributes:
        robots (Dict[str, SimulatedRobot]): Dictionary of simulated robot instances.

    ROS Parameters:
        sim_speed (double): Global speed multiplier for simulation (default: 1.0).

    ROS Topics:
        Publishes to:
            /fleet/fleet_state (fleet_interfaces/RobotStateArray): Real-time telemetry of the fleet.

    ROS Actions:
        Servers:
            /{robot_id}/navigate_to_pose (nav2_msgs/NavigateToPose): Simulates robot movement.
            /{robot_id}/compute_path_to_pose (nav2_msgs/ComputePathToPose): Simulates path planning.
    """

    def __init__(self) -> None:
        """
        Initializes the Mock Fleet Node, sets up simulated robots, and starts Action Servers.
        """
        super().__init__('mock_fleet')
        
        self.declare_parameter('sim_speed', 1.0)
        speed_factor = self.get_parameter('sim_speed').get_parameter_value().double_value

        # 1. Configuration de la flotte virtuelle (Vitesse ajoutée)
        self.robots: Dict[str, SimulatedRobot] = {
            "robot_1": SimulatedRobot("robot_1", x=0.0, y=0.5, battery=90.0, speed=speed_factor*0.5),
            "robot_2": SimulatedRobot("robot_2", x=1.0, y=1.0, battery=45.0, speed=speed_factor*0.3) 
        }

        self.cb_group = ReentrantCallbackGroup()

        # 2. Publisher d'état global
        self.state_pub = self.create_publisher(RobotStateArray, '/fleet/fleet_state', 10)
        self.create_timer(0.1, self.publish_fleet_state) # 5 Hz (plus fluide)

        # 3. Création des Action Servers
        self._action_servers: List[ActionServer] = []
        
        for r_id in self.robots:
            # NavigateToPose
            nav_topic = f"/{r_id}/navigate_to_pose"
            self._action_servers.append(ActionServer(
                self, NavigateToPose, nav_topic,
                execute_callback=lambda goal_handle, rid=r_id: self.navigate_callback(goal_handle, rid),
                callback_group=self.cb_group,
                cancel_callback=self.cancel_callback
            ))
            
            # ComputePathToPose
            plan_topic = f"/{r_id}/compute_path_to_pose"
            self._action_servers.append(ActionServer(
                self, ComputePathToPose, plan_topic,
                execute_callback=lambda goal_handle, rid=r_id: self.compute_path_callback(goal_handle, rid),
                callback_group=self.cb_group
            ))
            
        self.get_logger().info(f"Mock Fleet started with {len(self.robots)} robots.")

    def cancel_callback(self, goal_handle: Any) -> CancelResponse:
        """
        Callback to handle incoming cancellation requests for actions.

        Args:
            goal_handle (Any): The handle of the action goal being cancelled.

        Returns:
            CancelResponse: Always returns ACCEPT to allow cancellation.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_fleet_state(self) -> None:
        """
        Timer callback to publish the aggregated state of all simulated robots.

        Constructs a `RobotStateArray` message containing ID, battery, and pose
        for every robot in the simulation.
        """
        msg = RobotStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for r in self.robots.values():
            with r.lock:
                s = RobotState()
                s.robot_id = r.robot_id
                s.battery_pct = r.battery
                s.pose = r.get_pose()
                msg.robots.append(s)
        
        self.state_pub.publish(msg)

    def navigate_callback(self, goal_handle: Any, robot_id: str) -> Any:
        """
        Simulates the execution of a navigation action.

        This method blocks the thread to simulate travel time. It updates the robot's
        position incrementally towards the goal, calculates battery drain, and publishes feedback.

        Note:
            Uses `time.sleep()` to simulate real-world travel time.

        Args:
            goal_handle (Any): The action server goal handle.
            robot_id (str): The ID of the robot to move.

        Returns:
            NavigateToPose.Result: The final result of the action.
        """
        self.get_logger().info(f"[{robot_id}] Navigation started...")
        
        # Récupération de l'objet robot simulé
        robot = self.robots[robot_id]  # <--- Correction ici (c'était self.robots[r_id] dans la lambda, mais on le récupère proprement)
        
        target_pose = goal_handle.request.pose.pose
        tx, ty = target_pose.position.x, target_pose.position.y
        
        # Calcul du vecteur initial
        dx = tx - robot.x
        dy = ty - robot.y
        dist_total = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        
        with robot.lock:
            robot.status = "BUSY"
            robot.yaw = target_yaw
            
        rate = 0.1 # 10 Hz
        dist_traveled = 0.0
        
        # Boucle de mouvement
        while dist_traveled < dist_total:
            start_loop = time.time()
            
            # 1. Vérification Annulation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with robot.lock: robot.status = "IDLE"
                self.get_logger().info(f"[{robot_id}] Navigation canceled.")
                return NavigateToPose.Result()
            
            # 2. Avancée physique
            step_dist = robot.speed * rate
            
            # Ne pas dépasser
            if dist_traveled + step_dist > dist_total:
                step_dist = dist_total - dist_traveled
            
            dist_traveled += step_dist
            
            with robot.lock:
                robot.x += step_dist * math.cos(robot.yaw)
                robot.y += step_dist * math.sin(robot.yaw)
                robot.battery -= 0.05

            # 3. PUBLICATION DU FEEDBACK (CORRIGÉ)
            feedback_msg = NavigateToPose.Feedback()
            
            remaining_dist = dist_total - dist_traveled
            feedback_msg.distance_remaining = remaining_dist
            
            # --- CALCUL DU TEMPS RESTANT ---
            if robot.speed > 0:
                time_left_sec = remaining_dist / robot.speed
                feedback_msg.estimated_time_remaining.sec = int(time_left_sec)
                feedback_msg.estimated_time_remaining.nanosec = int((time_left_sec - int(time_left_sec)) * 1e9)
            else:
                # Si vitesse nulle, temps infini (ou très grand)
                feedback_msg.estimated_time_remaining.sec = 9999
            # --------------------------------------------
            
            goal_handle.publish_feedback(feedback_msg)
            
            # 4. Gestion du temps de boucle
            elapsed = time.time() - start_loop
            if elapsed < rate:
                time.sleep(rate - elapsed)
        
        # Fin
        goal_handle.succeed()
        with robot.lock:
            robot.x = tx
            robot.y = ty
            robot.status = "IDLE"
            
        self.get_logger().info(f"[{robot_id}] Arrived at ({tx}, {ty}).")
        return NavigateToPose.Result()

    def compute_path_callback(self, goal_handle: Any, robot_id: str) -> Any:
        """
        Simulates a path planner by generating a straight line path.

        This is a naive implementation that ignores obstacles, used purely
        for testing the upper layers of the architecture.

        Args:
            goal_handle (Any): The action server goal handle.
            robot_id (str): The ID of the robot requesting the plan.

        Returns:
            ComputePathToPose.Result: The result containing the computed path.
        """
        # Simulation d'un planificateur (A* simplifié en ligne droite)
        # On génère un chemin avec des points tous les 50cm pour être réaliste
        
        start_pose = goal_handle.request.start.pose if goal_handle.request.use_start else self.robots[robot_id].get_pose()
        goal_pose = goal_handle.request.goal.pose
        
        sx, sy = start_pose.position.x, start_pose.position.y
        gx, gy = goal_pose.position.x, goal_pose.position.y
        
        dist = math.hypot(gx - sx, gy - sy)
        step_size = 0.5 # 50cm
        num_steps = int(dist / step_size)
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Interpolation des points
        for i in range(num_steps + 1):
            alpha = i / max(1, num_steps)
            x = sx + (gx - sx) * alpha
            y = sy + (gy - sy) * alpha
            
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            
        # S'assurer que le dernier point est bien le goal
        final_ps = PoseStamped()
        final_ps.header = path.header
        final_ps.pose = goal_pose
        path.poses.append(final_ps)
        
        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = path
        # On peut aussi remplir result.planning_time pour plus de réalisme
        return result

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MockFleetNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()