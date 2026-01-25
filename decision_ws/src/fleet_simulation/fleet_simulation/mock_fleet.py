import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import math
import threading

# Messages
from fleet_interfaces.msg import RobotState, RobotStateArray
from geometry_msgs.msg import Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Path

class SimulatedRobot:
    def __init__(self, robot_id, x, y, battery=100.0):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.battery = battery
        self.status = "IDLE" # IDLE, BUSY
        self.lock = threading.RLock()

    def get_pose(self):
        p = Pose()
        p.position.x = float(self.x)
        p.position.y = float(self.y)
        p.orientation.w = 1.0 # Identité
        return p

class MockFleetNode(Node):
    def __init__(self):
        super().__init__('mock_fleet')
        
        # 1. Configuration de la flotte virtuelle
        self.robots = {
            "robot_1": SimulatedRobot("robot_1", x=0.0, y=0.0, battery=85.0),
            "robot_2": SimulatedRobot("robot_2", x=10.0, y=10.0, battery=45.0) # Batterie faible exprès
        }

        self.cb_group = ReentrantCallbackGroup()

        # 2. Publisher d'état global
        self.state_pub = self.create_publisher(RobotStateArray, '/fleet/fleet_state', 10)
        self.create_timer(0.5, self.publish_fleet_state) # 2 Hz

        # 3. Création des Action Servers pour CHAQUE robot
        self._action_servers = []
        
        for r_id in self.robots:
            # A. NavigateToPose
            nav_topic = f"/{r_id}/navigate_to_pose"
            self._action_servers.append(ActionServer(
                self, NavigateToPose, nav_topic,
                execute_callback=lambda goal_handle, rid=r_id: self.navigate_callback(goal_handle, rid),
                callback_group=self.cb_group
            ))
            
            # B. ComputePathToPose
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
                s.battery_pct = float(r.battery)
                s.pose = r.get_pose()
                # Tu peux ajouter d'autres champs ici selon ta définition de RobotState
                msg.robots.append(s)
        
        self.state_pub.publish(msg)

    def navigate_callback(self, goal_handle, robot_id):
        self.get_logger().info(f"[{robot_id}] Navigation request received.")
        robot = self.robots[robot_id]
        
        pose_stamped = goal_handle.request.pose
        target_x = pose_stamped.pose.position.x  # Ajout de .pose
        target_y = pose_stamped.pose.position.y  # Ajout de .pose
        
        # Simulation de déplacement
        with robot.lock:
            robot.status = "BUSY"
        
        # On simule un temps de trajet (ex: 1m/s)
        dist = math.hypot(target_x - robot.x, target_y - robot.y)
        duration = dist / 2.0 # On va vite (2m/s) pour les tests
        
        # Feedback loop
        steps = 10
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with robot.lock: robot.status = "IDLE"
                return NavigateToPose.Result()
            
            time.sleep(duration / steps)
            
            # Mise à jour position (Interpolation linéaire simple)
            with robot.lock:
                alpha = (i + 1) / steps
                robot.x = robot.x + (target_x - robot.x) * alpha
                robot.y = robot.y + (target_y - robot.y) * alpha
                robot.battery -= 0.1 # On consomme un peu
                
        goal_handle.succeed()
        with robot.lock:
            robot.status = "IDLE"
            
        self.get_logger().info(f"[{robot_id}] Arrived at destination.")
        return NavigateToPose.Result()

    def compute_path_callback(self, goal_handle, robot_id):
        # self.get_logger().info(f"[{robot_id}] Computing path...")
        
        # Pour ton BatchCostEstimator, on a besoin d'un Path valide.
        # On va tricher : on renvoie une ligne droite entre Start et Goal.
        
        start = goal_handle.request.start.pose if goal_handle.request.use_start else self.robots[robot_id].get_pose()
        goal = goal_handle.request.goal.pose
        
        path = Path()
        path.header.frame_id = "map"
        path.poses = [
            self._create_pose_stamped(start),
            self._create_pose_stamped(goal)
        ]
        
        goal_handle.succeed()
        result = ComputePathToPose.Result()
        result.path = path
        return result

    def _create_pose_stamped(self, pose):
        from geometry_msgs.msg import PoseStamped
        p = PoseStamped()
        p.pose = pose
        return p

def main(args=None):
    rclpy.init(args=args)
    node = MockFleetNode()
    
    # Executor multithread pour gérer plusieurs actions en parallèle
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