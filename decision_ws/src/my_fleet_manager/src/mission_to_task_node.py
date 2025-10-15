#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_fleet_manager.msg import Mission, Tache
from geometry_msgs.msg import Pose

class MissionToTaskNode(Node):
    def __init__(self):
        super().__init__('mission_to_task_node')

        # Publisher pour Tache
        self.task_publisher = self.create_publisher(Tache, 'task_topic', 10)

        # Subscriber pour Mission
        self.mission_subscriber = self.create_subscription(
            Mission,
            'mission_topic',
            self.mission_callback,
            10
        )
        self.get_logger().info('MissionToTaskNode initialized.')

        # Compteur de task_id
        self.task_id_counter = 0

    def mission_callback(self, msg: Mission):
        self.get_logger().info(f'Received Mission id={msg.mission_id} with {len(msg.points)} points')

        for i, point in enumerate(msg.points):
            tache_msg = Tache()
            tache_msg.task_id = self.task_id_counter
            tache_msg.mission_id = msg.mission_id
            tache_msg.task_type = "goto_point"
            tache_msg.goal = point

            self.task_publisher.publish(tache_msg)
            self.get_logger().info(f'Published Tache id={tache_msg.task_id} for point {i}')

            self.task_id_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionToTaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
