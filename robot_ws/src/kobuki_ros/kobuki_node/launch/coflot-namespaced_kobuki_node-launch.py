import os
import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import yaml

def generate_launch_description():

    # Solution for multi-robot systems used by Nav2, the stack used by CoFlot for navigation
    # according to : https://discourse.openrobotics.org/t/tf-tree-in-a-multi-robot-setup-in-ros2/41426
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
                  
    # --- 1. DÉCLARATION DU NAMESPACE ---
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace à appliquer au nœud et aux trames TF'
    )
    namespace = LaunchConfiguration('namespace')
    
    # --- Kobuki turtlebot2 default launchfile (with 1 change *) ---
                  
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_ros_node = launch_ros.actions.Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        output='both',
        # Apply namespace and remappings (pour les topics) (*)
        namespace=namespace, 
        remappings=remappings,
        parameters=[params])

    return launch.LaunchDescription([
        namespace_arg,
        kobuki_ros_node
    ])
