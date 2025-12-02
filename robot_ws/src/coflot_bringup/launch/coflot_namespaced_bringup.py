from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Solution for multi-robot systems used by Nav2, the stack used by CoFlot for navigation
    # according to : https://discourse.openrobotics.org/t/tf-tree-in-a-multi-robot-setup-in-ros2/41426    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
                  
    # --- Arguments ---
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Namespace du robot (e.g., robot1)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/delta3/launch/carte/salle2.yaml',
        description='Chemin du fichier carte YAML'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Utiliser le temps de simulation'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='Port série du RPLIDAR'
    )

    # --- Substitutions ---
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    
    # --- ACTIONS NAMESPACÉES (GroupAction) ---
    
    
    
    # --- Groupe pour les autres nœuds (RPLiDAR, CMD_VEL, ETAT) ---
    robot_namespaced_nodes = GroupAction([
        PushRosNamespace(robot_name),
        
        # 1- Kobuki base node (Hors PushRosNamespace car gère son propre namespace TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kobuki_node'),
                    'launch',
                    'coflot-namespaced_kobuki_node-launch.py' # Utilisez le nom de votre fichier corrigé
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time, 
            }.items()
        ),

        # 2- Robot description (URDF + TF) (Hors PushRosNamespace pour éviter le double prefix)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kobuki_description'),
                    'launch',
                    'coflot-robot_description_with_namespace.launch.py' # Utilisez le nom de votre fichier corrigé
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # 3- Command velocity node (Lit /robot1/cmd_vel)
        Node(
            package='kobuki_cmdvel',
            executable='kobuki_cmdvel',
            name='kobuki_cmdvel',
            output='screen'
        ),
        
        # 4- Static transform (base_link → laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            remappings=remappings,
            output='screen'
        ),
        
        # 5- RPLidar driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch',
                    'rplidar_a2m12_launch.py'
                )
            ),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_port'),
                # use_sim_time est géré en interne par le launchfile RPLiDAR ou son YAML
            }.items()
        ),

        # 6- Robot state node
        Node(
            package='kobuki_etat',
            executable='robot_etat_node.py',
            name='robot_etat_node',
            remappings=remappings,
            output='screen',
            parameters=[{'robot_name': robot_name}]
        ),
    ])

    # 7- Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': '/home/delta3/nav2_multirobot_params_1.yaml', # Chemin vers le YAML corrigé
            'use_namespace': 'True', # Indique au launchfile Nav2 d'utiliser le namespace
            'namespace': robot_name 
        }.items()
    )


    return LaunchDescription([

        # Lancement du groupe de nœuds namespacés
        robot_namespaced_nodes,
        
        # Lancement des nœuds Nav2
        nav2_launch,
    ])
