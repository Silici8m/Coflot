from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lancement du serveur web
        Node(
            package='web_manager',
            executable='web_server.py',
            name='web_server',
            output='screen'
        ),
        # Lancement de la réception des missions
        Node(
            package='web_manager',
            executable='web_reception.py',
            name='web_reception',
            output='screen'
        ),
        # Lancement de rosbridge websocket pour l'interface web
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge',
            output='screen',
            arguments=['--ros-args']
        )
    ])
