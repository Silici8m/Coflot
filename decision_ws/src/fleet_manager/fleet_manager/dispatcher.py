#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kobuki_etat.msg import RobotEtat
from my_fleet_manager.msg import Tache
import math
import queue
from geometry_msgs.msg import PoseStamped


class Dispatcher(Node):
    def __init__(self):
        super().__init__('dispatcher')

        self.task_queue = queue.Queue()

        # Abonnement au topic des tâches (Tache venant du séquenceur)
        self.subscription_task = self.create_subscription(
            Tache,
            'task_topic',
            self.listener_task_callback,
            10
        )

        # Abonnement au topic des états des robots
        self.subscription_state = self.create_subscription(
            RobotEtat,
            'robot_etat',
            self.listener_state_callback,
            10
        )

        # Publisher pour envoyer les missions assignées (PoseStamped)
        self.publisher_assignment = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # Dictionnaire pour stocker l'état des robots
        self.robots = {}

    # --- Callback pour réception des tâches ---
    def listener_task_callback(self, msg):
        self.task_queue.put(msg)
        self.get_logger().info(
            f"Tâche {msg.task_id} ajoutée à la file d’attente. Taille actuelle : {self.task_queue.qsize()}"
        )
        # Essayer d’assigner si un robot est dispo
        self.try_assign_tasks()

    # --- Callback pour réception de l’état des robots ---
    def listener_state_callback(self, msg):
        self.robots[msg.robot_id] = {
            'x': msg.x,
            'y': msg.y,
            'mode': msg.mode
        }

        # Si le robot est dispo, essayer d’assigner une tâche
        if msg.mode == "idle":
            self.try_assign_tasks()

    # --- Fonction d'assignation ---
    def try_assign_tasks(self):
        if self.task_queue.empty():
            return

        # Trouver les robots libres
        available_robots = [r for r, s in self.robots.items() if s['mode'] == "idle"]
        if not available_robots:
            print("try_assign aucun robot")        	
            return  # aucun robot dispo, on attend

        # On essaie d’assigner autant de tâches que possible
        for robot_id in available_robots:
            if self.task_queue.empty():
                break

            task_msg = self.task_queue.get()
            goal = task_msg.goal

            # Envoi au robot
            msg_out = PoseStamped()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = "map"
            msg_out.pose = goal

            self.publisher_assignment.publish(msg_out)
            self.robots[robot_id]['mode'] = "busy"
            self.get_logger().info(
                f"Tâche {task_msg.task_id} assignée à {robot_id}. Tâches restantes : {self.task_queue.qsize()}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Dispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

