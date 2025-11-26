#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kobuki_etat.msg import RobotEtat
from my_fleet_manager.msg import Tache
import math

from geometry_msgs.msg import PoseStamped


class Dispatcher(Node):
    def __init__(self):
        super().__init__('dispatcher')

        # Abonnement au topic des tâches (Tache venant du séquenceur)
        self.subscription_task = self.create_subscription(
            Tache,
            'task_topic',
            self.listener_task_callback,
            10)

        # Abonnement au topic des états des robots
        self.subscription_state = self.create_subscription(
            RobotEtat,
            'robot_etat',
            self.listener_state_callback,
            10)

        # Publisher pour envoyer les missions assignées (PoseStamped)
        self.publisher_assignment = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)

        # Dictionnaire pour stocker l'état des robots
        self.robots = {}

    def listener_task_callback(self, msg):
        # Récupération du goal depuis le message Tache
        goal = msg.goal
        position = goal.position
        orientation = goal.orientation

        # Conversion quaternion -> yaw (pour log)
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        )

        self.get_logger().info(
            f"Tâche {msg.task_id} (mission {msg.mission_id}, type {msg.task_type}) "
            f"reçue à position ({position.x:.2f}, {position.y:.2f}), orientation {yaw:.2f} rad"
        )

        # --- Choix du robot le plus proche disponible ---
        robot_choice = None
        min_distance = float('inf')

        for robot, state in self.robots.items():
            if state['mode'] == "idle":  # robot disponible
                dist = math.sqrt((position.x - state['x'])**2 + (position.y - state['y'])**2)
                if dist < min_distance:
                    min_distance = dist
                    robot_choice = robot

        # --- Publier l’assignation ---
        if robot_choice:
            self.get_logger().info(f"Assignation de la tâche {msg.task_id} au robot {robot_choice}")

            msg_out = PoseStamped()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = "map"  # à adapter selon ton frame de navigation
            msg_out.pose.position = position
            msg_out.pose.orientation = orientation

            self.publisher_assignment.publish(msg_out)

            # Marquer le robot comme occupé
            self.robots[robot_choice]['mode'] = "busy"
        else:
            self.get_logger().warn("Aucun robot disponible pour cette tâche")

    def listener_state_callback(self, msg):
        """
        Callback qui reçoit un RobotEtat et met à jour l'état du robot dans le dictionnaire interne.
        """
        robot_id = msg.robot_id
        mode = msg.mode              # "idle", "busy", "error", "charging"
        battery = msg.battery_pct    # en %
        x = msg.x
        y = msg.y
        yaw = msg.yaw
        stamp = msg.stamp            # float64 Unix time

        # Mise à jour du dictionnaire interne
        self.robots[robot_id] = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'mode': mode,
            'battery': battery,
            'stamp': stamp
        }

        # Log d'information
        self.get_logger().info(
            f"[{stamp:.1f}] {robot_id} -> pos=({x:.2f}, {y:.2f}, yaw={yaw:.2f}) "
            f"mode={mode} battery={battery:.1f}%"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Dispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
