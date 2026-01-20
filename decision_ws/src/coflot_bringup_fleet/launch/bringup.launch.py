from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Chemins
    pkg_dir = get_package_share_directory('coflot_bringup_fleet')
    
    # --- Arguments ---
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'salle2.yaml'),
        description='Chemin vers le fichier YAML de la map'
    )

    return LaunchDescription([
        map_yaml_arg,

        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'topic_name': 'map',
                'frame_id': 'map'
            }]
        ),

        # 2. Lifecycle Manager (Activation automatique)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # 3. Rosbridge WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        )
    ])