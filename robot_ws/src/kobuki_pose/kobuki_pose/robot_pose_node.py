#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tf2_ros
import math
import time


class RobotPoseNode(Node):
    def __init__(self):
        super().__init__('robot_pose_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(Pose, 'robot_pose', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer pour publier régulièrement l’état
        self.timer = self.create_timer(0.1, self.publish_robot_state)

        self.get_logger().info("✅ robot_etat_node lancé (publie sur /robot_etat)")

    # ---------------------- Callbacks ----------------------


    def get_robot_pose(self):
        """Lecture de la position du robot via TF et retour d'un objet Pose."""
        try:
            # Récupération de la transformation
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Création du message Pose
            pose = Pose()
            
            # Position
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            
            # Orientation (on copie directement le quaternion)
            pose.orientation = trans.transform.rotation

            return pose

        except Exception as e:
            now = time.time()
            if now - getattr(self, 'last_tf_warn_time', 0) > 5.0:
                self.get_logger().warn(f"⚠ Impossible de lire la TF map->base_link : {e}")
                self.last_tf_warn_time = now
            return None

    def publish_robot_state(self):
        """Publication de l’état du robot."""
        pose = self.get_robot_pose()
        if pose is None:
            return

        self.publisher_.publish(pose)

        self.get_logger().info(
            f"x={pose.position.x:.2f}, y={pose.position.y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



