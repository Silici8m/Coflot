#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_fleet_manager.msg import Mission
from geometry_msgs.msg import Pose

class WebReception(Node):
    def __init__(self):
        super().__init__('web_reception')
        self.publisher_ = self.create_publisher(Mission, 'mission_topic', 10)
        self.mission_id = 0
        self.get_logger().info("Nœud web_reception prêt à recevoir des missions.")

    def publish_mission(self, points):
        """Publie une mission avec les points reçus du front-end"""
        msg = Mission()
        self.mission_id += 1
        msg.mission_id = self.mission_id
        msg.num_points = len(points)

        msg.points = []
        for p in points:
            pose = Pose()
            pose.position.x = p['x']
            pose.position.y = p['y']
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.points.append(pose)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Mission {msg.mission_id} publiée avec {msg.num_points} points.")

def main(args=None):
    rclpy.init(args=args)
    node = WebReception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
