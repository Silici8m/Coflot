#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from kobuki_etat.msg import RobotEtat
from nav2_msgs.msg import BehaviorTreeLog
import tf2_ros
import math
import time


class RobotEtatNode(Node):
    def __init__(self):
        super().__init__('robot_etat_node')
        
        self.declare_parameter('robot_name', 'robot1')

        # Publisher
        self.publisher_ = self.create_publisher(RobotEtat, '/robot_etat', 10)

        # Battery
        self.battery_percent = None
        self.create_subscription(BatteryState, 'sensors/battery_state', self.battery_callback, 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Abonnement au Behavior Tree log (pour suivre l’état de Nav2)
        self.nav_status = "IDLE"
        self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.behavior_tree_callback,
            10
        )

        # Timer pour publier régulièrement l’état
        self.timer = self.create_timer(1.0, self.publish_robot_state)

        # Pour éviter le spam de logs TF
        self.last_tf_warn_time = 0.0

        self.get_logger().info("✅ robot_etat_node lancé (publie sur /robot_etat)")

    # ---------------------- Callbacks ----------------------

    def battery_callback(self, msg):
        """Mise à jour du niveau de batterie."""
        if not math.isnan(msg.percentage):
            self.battery_percent = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        elif msg.voltage > 0.0:
            self.battery_percent = min(max((msg.voltage - 12.0) / (16.5 - 12.0) * 100.0, 0.0), 100.0)
        else:
            self.battery_percent = 0.0

    def behavior_tree_callback(self, msg: BehaviorTreeLog):
        """Analyse du log Nav2 pour savoir si le robot navigue, a terminé, ou a échoué."""
        if not msg.event_log:
            return

        # On prend le dernier event
        last_event = msg.event_log[-1]
        self.nav_status = last_event.current_status  # e.g. RUNNING, SUCCESS, FAILURE

        if self.nav_status == "RUNNING":
            self.get_logger().info("🤖 Navigation en cours...")
        elif self.nav_status == "SUCCESS":
            self.get_logger().info("🏁 Navigation terminée avec succès.")
        elif self.nav_status == "FAILURE":
            self.get_logger().warn("⚠️ Échec de la navigation.")

    def get_robot_pose(self):
        """Lecture de la position du robot via TF."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return x, y, yaw
        except Exception as e:
            now = time.time()
            if now - self.last_tf_warn_time > 5.0:
                self.get_logger().warn(f"⚠ Impossible de lire la TF map->base_link : {e}")
                self.last_tf_warn_time = now
            return None, None, None

    def publish_robot_state(self):
        """Publication de l’état du robot."""
        x, y, yaw = self.get_robot_pose()
        if x is None or self.battery_percent is None:
            return  # attend d’avoir la pose ET la batterie

        msg = RobotEtat()
        msg.robot_id = self.get_parameter('robot_name').get_parameter_value().string_value

        # ✅ Détermine le mode à partir du Behavior Tree
        if self.nav_status == "RUNNING":
            msg.mode = "busy"
        else:
            msg.mode = "idle"

        msg.battery_pct = float(self.battery_percent)
        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)
        msg.stamp = time.time()

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"[PUB] id={msg.robot_id}, mode={msg.mode}, "
            f"x={msg.x:.2f}, y={msg.y:.2f}, yaw={math.degrees(msg.yaw):.1f}°, "
            f"batt={msg.battery_pct:.1f}%, nav_status={self.nav_status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotEtatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



