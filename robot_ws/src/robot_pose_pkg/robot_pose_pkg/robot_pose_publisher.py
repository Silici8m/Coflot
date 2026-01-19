import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Déclaration des paramètres avec valeurs par défaut
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Récupération des valeurs
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Pose, 'robot_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info(f"Ecoute TF: {self.map_frame} -> {self.base_frame}")

    def publish_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            
            msg = Pose()
            msg.position.x = trans.transform.translation.x
            msg.position.y = trans.transform.translation.y
            msg.position.z = trans.transform.translation.z
            msg.orientation = trans.transform.rotation
            
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"TF manquant ({self.map_frame}->{self.base_frame}) : {e}", throttle_duration_sec=5.0)

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