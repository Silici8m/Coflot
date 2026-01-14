from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    kobuki_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kobuki_node'),
                'launch',
                'kobuki_node-launch.py'
            )
        )
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m12_launch.py'
            )
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )
    
    kobuki_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch',
                'robot_description.launch.py'
            )
        )
    )

    return LaunchDescription([
        kobuki_node_launch,
        kobuki_desc_launch,

        Node(
            package='kobuki_cmdvel',
            executable='kobuki_cmdvel',
            name='kobuki_cmdvel',
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        rplidar_launch,

        Node(
            package='kobuki_etat',
            executable='robot_etat_node.py',
            name='robot_etat_node',
            output='screen'
        ),
    ])



