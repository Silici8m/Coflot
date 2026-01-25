import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

import time
import math
import threading

# Messages
from fleet_interfaces.msg import RobotState, RobotStateArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Path

def euler_to_quaternion(yaw):
    """Convertit un angle yaw (radians) en Quaternion ROS"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class SimulatedRobot:
    def __init__(self, robot_id, x, y, battery=100.0, speed=1.0):
        self.robot_id = robot_id
        self.x = float(x)
        self.y = float(y)
        self.yaw = 0.0 # Orientation en radians
        self.battery = float(battery)
        self.speed = speed # m/s
        self.status = "IDLE" 
        self.lock = threading.RLock()

    def get_pose(self):
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.orientation = euler_to_quaternion(self.yaw)
        return p

class MockFleetNode(Node):
    def __init__(self):
        super().__init__('mock_fleet')
        
        # 1. Configuration de la flotte virtuelle (Vitesse ajoutée)
        self.robots = {
            "robot_1": SimulatedRobot("robot_1", x=0.0, y=0.5, battery=90.0, speed=0.5),
            "robot_2": SimulatedRobot("robot_2", x=1.0, y=1.0, battery=45.0, speed=0.3) 
        }

        self.cb_group = ReentrantCallbackGroup()

        # 2. Publisher d'état global
        self.state_pub = self.create_publisher(RobotStateArray, '/fleet/fleet_state', 10)
        self.create_timer(0.1, self.publish_fleet_state) # 5 Hz (plus fluide)

        # 3. Création des Action Servers
        self._action_servers = []
        
        for r_id in self.robots:
            # NavigateToPose
            nav_topic = f"/{r_id}/navigate_to_pose"
            self._action_servers.append(ActionServer(
                self, NavigateToPose, nav_topic,
                execute_callback=lambda goal_handle, rid=r_id: self.navigate_callback(goal_handle, rid),
                callback_group=self.cb_group
            ))
            
            # ComputePathToPose
            plan_topic = f"/{r_id}/compute_path_to_pose"
            self._action_servers.append(ActionServer(
                self, ComputePathToPose, plan_topic,
                execute_callback=lambda goal_handle, rid=r_id: self.compute_path_callback(goal_handle, rid),
                callback_group=self.cb_group
            ))
            
        self.get_logger().info(f"Mock Fleet started with {len(self.robots)} robots.")

    def publish_fleet_state(self):
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

    def navigate_callback(self, goal_handle, robot_id):
        self.get_logger().info(f"[{robot_id}] Navigation started...")
        robot = self.robots[robot_id]
        
        target_pose = goal_handle.request.pose.pose
        tx, ty = target_pose.position.x, target_pose.position.y
        
        with robot.lock:
            robot.status = "BUSY"
            
        # 1. Calcul de la distance totale et du vecteur
        dx = tx - robot.x
        dy = ty - robot.y
        dist_total = math.hypot(dx, dy)
        
        # 2. Orientation vers la cible
        target_yaw = math.atan2(dy, dx)
        with robot.lock:
            robot.yaw = target_yaw
            
        # 3. Boucle de simulation (10 Hz)
        rate = 0.1 # secondes par tick
        dist_traveled = 0.0
        
        while dist_traveled < dist_total:
            # Vérification Annulation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with robot.lock: robot.status = "IDLE"
                self.get_logger().info(f"[{robot_id}] Navigation canceled.")
                return NavigateToPose.Result()
            
            # Simulation Physique
            start_time = time.time()
            
            # Avancer selon la vitesse du robot
            step_dist = robot.speed * rate
            
            # Ne pas dépasser la cible
            if dist_traveled + step_dist > dist_total:
                step_dist = dist_total - dist_traveled
            
            with robot.lock:
                robot.x += step_dist * math.cos(robot.yaw)
                robot.y += step_dist * math.sin(robot.yaw)
                robot.battery -= 0.05 # Consommation
                
            dist_traveled += step_dist
            
            # Publication du Feedback (Distance restante)
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.distance_remaining = dist_total - dist_traveled
            goal_handle.publish_feedback(feedback_msg)
            
            # Attente pour respecter le taux de rafraichissement
            elapsed = time.time() - start_time
            if elapsed < rate:
                time.sleep(rate - elapsed)
        
        # Fin du mouvement
        goal_handle.succeed()
        with robot.lock:
            robot.x = tx # On force la position exacte à la fin pour éviter les erreurs d'arrondi
            robot.y = ty
            robot.status = "IDLE"
            
        self.get_logger().info(f"[{robot_id}] Arrived at ({tx}, {ty}).")
        return NavigateToPose.Result()

    def compute_path_callback(self, goal_handle, robot_id):
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

def main(args=None):
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