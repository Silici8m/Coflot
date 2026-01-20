from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # 1. Declare and retrieve parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 2. Setup Publisher and TF Buffer
        self.publisher_ = self.create_publisher(Pose, 'robot_pose', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info(f"Listening for TF: {self.map_frame} -> {self.base_frame}")

    def publish_pose(self):
        # Use instance variables by default
        pose_stamped = self.get_robot_pose(self.base_frame, self.map_frame)
        
        if pose_stamped:
            # We publish the 'Pose' part as per your publisher definition
            self.publisher_.publish(pose_stamped.pose)

    def get_robot_pose(self, base_frame: str, map_frame: str) -> Optional[PoseStamped]:
        """Fetches the transform and converts it to PoseStamped."""
        try:
            # lookup_transform uses (target_frame, source_frame, time)
            transform = self.tf_buffer.lookup_transform(
                map_frame, 
                base_frame, 
                Time(), 
                timeout=Duration(seconds=0.1) # Reduced timeout for 10Hz loop
            )

            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            return pose_stamped

        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=5.0)
            return None

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()