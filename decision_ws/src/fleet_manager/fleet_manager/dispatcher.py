#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from kobuki_etat.msg import RobotEtat
from my_fleet_manager.msg import Tache
from geometry_msgs.msg import PoseStamped
from enum import Enum
import math

class RobotMode(Enum):
    IDLE = 'idle'
    BUSY = 'busy'
    ERROR = 'error'
    CHARGING = 'charging'
    UNKNOWN = 'unknown'

class RobotData:
    def __init__(self, robot_id: str, publisher, clock):
        self.robot_id = robot_id
        self.mode = RobotMode.UNKNOWN 
        self.last_update = Time(seconds=0, nanoseconds=0, clock_type=clock.clock_type)
        self.publisher = publisher
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.battery = 0.0

    def update_state(self, msg: RobotEtat, current_time: Time):
        try:
            self.mode = RobotMode(msg.mode.lower())
        except ValueError:
            self.mode = RobotMode.UNKNOWN
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw
        self.battery = msg.battery_pct
        self.last_update = current_time

class Dispatcher(Node):
    def __init__(self):
        super().__init__('dispatcher')
        
        self.robots = {}
        self.waiting_tasks = [] 
        self.inactivity_timeout = Duration(seconds=5, nanoseconds=0)

        self.create_subscription(Tache, 'task_topic', self.listener_task_callback, 10)
        self.create_subscription(RobotEtat, 'robot_etat', self.listener_state_callback, 10)
        self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Dispatcher démarré")

    def listener_state_callback(self, msg: RobotEtat):
        robot_id = msg.robot_id
        current_time = self.get_clock().now()
        topic_name = f"{robot_id}/goal_pose"

        if robot_id not in self.robots:
            publisher = self.create_publisher(PoseStamped, topic_name, 10)
            self.robots[robot_id] = RobotData(robot_id, publisher, self.get_clock())
            self.get_logger().info(f"Connexion du robot : {robot_id}")

        # Capture de l'ancien mode pour comparaison
        robot = self.robots[robot_id]
        old_mode = robot.mode

        # Mise à jour
        robot.update_state(msg, current_time)

        # Log si changement
        if old_mode != robot.mode:
            self.get_logger().info(f"Robot {robot_id} : {old_mode.value} -> {robot.mode.value}")

    def listener_task_callback(self, msg: Tache):
        self.waiting_tasks.append(msg)
        self.get_logger().info(f"Tâche {msg.task_id} ajoutée à la file d'attente (Total: {len(self.waiting_tasks)})")

    def timer_callback(self):
        self.update_fleet()
        self.process_waiting_tasks()

    def process_waiting_tasks(self):
        if not self.waiting_tasks:
            return

        unassigned_tasks = []

        for task in self.waiting_tasks:
            target_pose = task.goal
            robot_choice = None
            min_distance = float('inf')

            for robot_id, data in self.robots.items():
                if data.mode == RobotMode.IDLE:
                    dist = math.sqrt(
                        (target_pose.position.x - data.x)**2 + (target_pose.position.y - data.y)**2
                    )
                    if dist < min_distance:
                        min_distance = dist
                        robot_choice = robot_id

            if robot_choice:
                self.get_logger().info(f"Tâche {task.task_id} assignée au robot {robot_choice}")
                
                msg_out = PoseStamped()
                msg_out.header.stamp = self.get_clock().now().to_msg()
                msg_out.header.frame_id = "map"
                msg_out.pose = target_pose

                self.robots[robot_choice].publisher.publish(msg_out)
                
                # Changement manuel du mode et Log
                old_mode = self.robots[robot_choice].mode
                self.robots[robot_choice].mode = RobotMode.BUSY
                self.get_logger().info(f"Robot {robot_choice} : {old_mode.value} -> {RobotMode.BUSY.value}")
            else:
                unassigned_tasks.append(task)
        
        self.waiting_tasks = unassigned_tasks
        
        if self.waiting_tasks:
            self.get_logger().info(f"{len(self.waiting_tasks)} tâches en attente")

    def update_fleet(self):
        current_time = self.get_clock().now()
        to_remove = []

        for robot_id, data in self.robots.items():
            if (current_time - data.last_update) > self.inactivity_timeout:
                to_remove.append(robot_id)

        for robot_id in to_remove:
            if self.robots[robot_id].publisher:
                self.destroy_publisher(self.robots[robot_id].publisher)
            del self.robots[robot_id]
            self.get_logger().warn(f"Robot {robot_id} supprimé (inactif).")

def main(args=None):
    rclpy.init(args=args)
    node = Dispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for data in node.robots.values():
            node.destroy_publisher(data.publisher)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()