#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import math

# Messages standards
from sensor_msgs.msg import BatteryState
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import Pose

# Messages Custom
from fleet_interfaces.msg import RobotState, RobotStateArray

FLEET_STATE_TOPIC = '/fleet/fleet_state'
POSE_TIMEOUT_SEC = 2.0  # Temps avant de warn

class FleetStateAggregator(Node):
    def __init__(self):
        super().__init__('fleet_adapter')

        self.fleet_pub = self.create_publisher(RobotStateArray, FLEET_STATE_TOPIC, 10)

        # Structure : { 'robot_id': { 'battery': float, 'mode': str, 'pose': Pose, 'last_pose_time': Time } }
        self.robots_data = {}
        self.known_robots = set()
        self.subscribers = [] 

        self.create_timer(1.0, self.scan_for_robots)
        self.create_timer(0.1, self.publish_fleet_state)

        self.get_logger().info("Fleet State Aggregator (Mode Pose Topic) démarré.")

    def scan_for_robots(self):
        topic_names_and_types = self.get_topic_names_and_types()
        active_robots_in_scan = set()

        for topic_name, types in topic_names_and_types:
            if 'sensor_msgs/msg/BatteryState' in types and topic_name.endswith('/sensors/battery_state'):
                parts = topic_name.strip('/').split('/')
                if len(parts) >= 3:
                    robot_id = parts[0]
                    if self.count_publishers(topic_name) > 0:
                        active_robots_in_scan.add(robot_id)
                        if robot_id not in self.known_robots:
                            self.register_new_robot(robot_id)

        disconnected_robots = self.known_robots - active_robots_in_scan
        for robot_id in disconnected_robots:
            self.unregister_robot(robot_id)

    def register_new_robot(self, robot_id):
        self.get_logger().info(f"✨ Connexion du robot : {robot_id}")
        self.known_robots.add(robot_id)

        self.robots_data[robot_id] = {
            'battery': 0.0,
            'mode': 'idle',
            'nav_status': 'IDLE',
            'pose': Pose(),
            # On initialise le timer au moment de la connexion
            'last_pose_time': self.get_clock().now() 
        }

        # 1. Sub Batterie
        sub_bat = self.create_subscription(
            BatteryState, f"/{robot_id}/sensors/battery_state",
            partial(self.battery_callback, robot_id=robot_id), 10
        )
        self.subscribers.append(sub_bat)

        # 2. Sub Behavior Tree
        sub_bt = self.create_subscription(
            BehaviorTreeLog, f"/{robot_id}/behavior_tree_log",
            partial(self.bt_log_callback, robot_id=robot_id), 10
        )
        self.subscribers.append(sub_bt)

        # 3. Sub Pose
        sub_pose = self.create_subscription(
            Pose, f"/{robot_id}/robot_pose",
            partial(self.pose_callback, robot_id=robot_id), 10
        )
        self.subscribers.append(sub_pose)

    def unregister_robot(self, robot_id):
        self.get_logger().info(f"💤 Déconnexion du robot : {robot_id}")
        if robot_id in self.known_robots:
            self.known_robots.remove(robot_id)
        if robot_id in self.robots_data:
            del self.robots_data[robot_id]

    # ---------------------- Callbacks ----------------------

    def battery_callback(self, msg: BatteryState, robot_id: str):
        if robot_id in self.robots_data:
            pct = 0.0
            if not math.isnan(msg.percentage):
                pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
            elif msg.voltage > 0.0:
                pct = min(max((msg.voltage - 12.0) / (16.5 - 12.0) * 100.0, 0.0), 100.0)
            self.robots_data[robot_id]['battery'] = float(pct)

    def bt_log_callback(self, msg: BehaviorTreeLog, robot_id: str):
        if robot_id in self.robots_data and msg.event_log:
            status = msg.event_log[-1].current_status
            self.robots_data[robot_id]['nav_status'] = status
            self.robots_data[robot_id]['mode'] = "busy" if status == "RUNNING" else "idle"

    def pose_callback(self, msg: Pose, robot_id: str):
        """Met à jour la pose et le timestamp."""
        if robot_id in self.robots_data:
            self.robots_data[robot_id]['pose'] = msg
            # Mise à jour du timestamp
            self.robots_data[robot_id]['last_pose_time'] = self.get_clock().now()

    # ---------------------- Publish ----------------------

    def publish_fleet_state(self):
        if not self.known_robots:
            return

        msg_array = RobotStateArray()
        current_time = self.get_clock().now()
        
        msg_array.header.stamp = current_time.to_msg()
        msg_array.header.frame_id = "map"

        for robot_id in self.known_robots:
            if robot_id not in self.robots_data: continue
            data = self.robots_data[robot_id]

            # --- CHECK TIMEOUT POSE ---
            # Calcul du delta temps en secondes
            time_diff = (current_time - data['last_pose_time']).nanoseconds / 1e9
            
            if time_diff > POSE_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"⚠️ Pas de données de pose pour {robot_id} depuis {time_diff:.1f}s",
                    throttle_duration_sec=2.0
                )
            # --------------------------

            robot_state = RobotState()
            robot_state.robot_id = robot_id
            robot_state.mode = data['mode']
            robot_state.battery_pct = float(data['battery'])
            robot_state.pose = data['pose']
            robot_state.stamp = current_time.to_msg()
            msg_array.robots.append(robot_state)

        self.fleet_pub.publish(msg_array)


def main(args=None):
    rclpy.init(args=args)
    node = FleetStateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()