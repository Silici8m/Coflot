#!/usr/bin/env python3
# fleet_adapter.py

"""Fleet State Aggregator Node.

This module defines the FleetStateAggregator node, which centralizes information 
from multiple robots (battery, pose, navigation status) and publishes a unified 
fleet state message.
"""

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.time import Time
from functools import partial
import math
from typing import List, Dict, Set, Optional, Any, Tuple

# Standard Messages
from sensor_msgs.msg import BatteryState
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import Pose

# Custom Messages
from fleet_interfaces.msg import RobotState, RobotStateArray

FLEET_STATE_TOPIC: str = '/fleet/fleet_state'
POSE_TIMEOUT_SEC: float = 2.0  # Time before warning

class FleetStateAggregator(Node):
    """Aggregates states from multiple robots into a single fleet message.

    This node dynamically scans for active robots based on their topic namespaces,
    subscribes to their individual status topics (battery, behavior tree, pose),
    and publishes a consolidated `RobotStateArray`.

    Attributes:
        robots_data (Dict[str, Dict[str, Any]]): Storage for the latest data 
            of each robot. keys are robot_ids. 
            Structure: { 'robot_id': { 'battery': float, 'mode': str, 
            'pose': Pose, 'last_pose_time': Time } }.
        known_robots (Set[str]): Set of currently registered robot IDs.
        subscribers (List[Subscription]): List of active subscription objects 
            to prevent garbage collection.

    ROS Topics:
        Publishers:
            /fleet/fleet_state (fleet_interfaces/msg/RobotStateArray): 
                The aggregated state of the entire fleet.
        
        Subscribers (Dynamically created per robot_id):
            /{robot_id}/sensors/battery_state (sensor_msgs/msg/BatteryState): 
                Battery level.
            /{robot_id}/behavior_tree_log (nav2_msgs/msg/BehaviorTreeLog): 
                Navigation status.
            /{robot_id}/robot_pose (geometry_msgs/msg/Pose): 
                Current robot position.
    """

    def __init__(self) -> None:
        """Initializes the FleetStateAggregator node."""
        super().__init__('fleet_adapter')

        self.fleet_pub = self.create_publisher(RobotStateArray, FLEET_STATE_TOPIC, 10)

        # Structure : { 'robot_id': { 'battery': float, 'mode': str, 'pose': Pose, 'last_pose_time': Time } }
        self.robots_data: Dict[str, Dict[str, Any]] = {}
        self.known_robots: Set[str] = set()
        self.subscribers: List[Subscription] = [] 

        self.create_timer(1.0, self.scan_for_robots)
        self.create_timer(0.1, self.publish_fleet_state)

        self.get_logger().info("Fleet State Aggregator (Mode Pose Topic) démarré.")

    def scan_for_robots(self) -> None:
        """Scans the ROS graph to detect new or disconnected robots.

        This method looks for topics ending with '/sensors/battery_state'. 
        If a topic matches, the prefix is assumed to be the robot_id.
        It registers new robots and unregisters those that are no longer active.
        """
        topic_names_and_types: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()
        active_robots_in_scan: Set[str] = set()

        for topic_name, types in topic_names_and_types:
            if 'sensor_msgs/msg/BatteryState' in types and topic_name.endswith('/sensors/battery_state'):
                parts: List[str] = topic_name.strip('/').split('/')
                if len(parts) >= 3:
                    robot_id: str = parts[0]
                    if self.count_publishers(topic_name) > 0:
                        active_robots_in_scan.add(robot_id)
                        if robot_id not in self.known_robots:
                            self.register_new_robot(robot_id)

        disconnected_robots: Set[str] = self.known_robots - active_robots_in_scan
        for robot_id in disconnected_robots:
            self.unregister_robot(robot_id)

    def register_new_robot(self, robot_id: str) -> None:
        """Registers a new robot and subscribes to its topics.

        Args:
            robot_id (str): The unique identifier of the robot (namespace).
        """
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

    def unregister_robot(self, robot_id: str) -> None:
        """Unregisters a robot and cleans up its data.

        Note:
            This implementation does not explicitly destroy individual subscriptions 
            stored in the `subscribers` list, relying on Python's garbage collection 
            or node destruction if necessary.

        Args:
            robot_id (str): The unique identifier of the robot to remove.
        """
        self.get_logger().info(f"💤 Déconnexion du robot : {robot_id}")
        if robot_id in self.known_robots:
            self.known_robots.remove(robot_id)
        if robot_id in self.robots_data:
            del self.robots_data[robot_id]

    # ---------------------- Callbacks ----------------------

    def battery_callback(self, msg: BatteryState, robot_id: str) -> None:
        """Updates the battery level for a specific robot.

        Handles both percentage-based (0.0-1.0) and voltage-based reporting 
        (estimated mapping from 12V to 16.5V).

        Args:
            msg (BatteryState): The battery state message.
            robot_id (str): The ID of the robot.
        """
        if robot_id in self.robots_data:
            pct: float = 0.0
            if not math.isnan(msg.percentage):
                pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
            elif msg.voltage > 0.0:
                pct = min(max((msg.voltage - 12.0) / (16.5 - 12.0) * 100.0, 0.0), 100.0)
            self.robots_data[robot_id]['battery'] = float(pct)

    def bt_log_callback(self, msg: BehaviorTreeLog, robot_id: str) -> None:
        """Updates the navigation status and mode based on Behavior Tree logs.

        Args:
            msg (BehaviorTreeLog): The behavior tree log message.
            robot_id (str): The ID of the robot.
        """
        if robot_id in self.robots_data and msg.event_log:
            status = msg.event_log[-1].current_status
            self.robots_data[robot_id]['nav_status'] = status
            self.robots_data[robot_id]['mode'] = "busy" if status == "RUNNING" else "idle"

    def pose_callback(self, msg: Pose, robot_id: str) -> None:
        """Updates the pose and the last received timestamp for a robot.

        Args:
            msg (Pose): The pose message.
            robot_id (str): The ID of the robot.
        """
        if robot_id in self.robots_data:
            self.robots_data[robot_id]['pose'] = msg
            # Mise à jour du timestamp
            self.robots_data[robot_id]['last_pose_time'] = self.get_clock().now()

    # ---------------------- Publish ----------------------

    def publish_fleet_state(self) -> None:
        """Aggregates and publishes the state of all known robots.

        This method checks for data staleness (timeout) on robot poses and 
        issues a warning if data is too old.
        """
        if not self.known_robots:
            return

        msg_array = RobotStateArray()
        current_time: Time = self.get_clock().now()
        
        msg_array.header.stamp = current_time.to_msg()
        msg_array.header.frame_id = "map"

        for robot_id in self.known_robots:
            if robot_id not in self.robots_data: continue
            data: Dict[str, Any] = self.robots_data[robot_id]

            # --- CHECK TIMEOUT POSE ---
            # Calcul du delta temps en secondes
            last_pose_time: Time = data['last_pose_time']
            time_diff: float = (current_time - last_pose_time).nanoseconds / 1e9
            
            if time_diff > POSE_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"⚠️ Pas de données de pose pour {robot_id} depuis {time_diff:.1f}s",
                    throttle_duration_sec=2.0
                )
            # --------------------------

            robot_state = RobotState()
            robot_state.robot_id = robot_id
            robot_state.mode = str(data['mode'])
            robot_state.battery_pct = float(data['battery'])
            robot_state.pose = data['pose']
            robot_state.stamp = current_time.to_msg()
            msg_array.robots.append(robot_state)

        self.fleet_pub.publish(msg_array)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the fleet state aggregator node.

    Args:
        args (Optional[List[str]]): Command line arguments.
    """
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