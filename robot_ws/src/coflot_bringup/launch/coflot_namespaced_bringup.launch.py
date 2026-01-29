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

    # Get the launch directory
    bringup_dir = get_package_share_directory('coflot_bringup')
                  
    # --- Arguments ---
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Namespace du robot (e.g., robot1)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(bringup_dir, 'map', 'salle2.yaml'),
        description='Chemin du fichier carte YAML'
    )

    params_nav2_arg = DeclareLaunchArgument(
        'params',
        default_value= os.path.join(bringup_dir, 'params', 'coflot_nav2_multirobot_params.yaml'),
        description='Chemin du fichier params'
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
    
    kobuki_port_arg = DeclareLaunchArgument(
        'kobuki_port',
        default_value='/dev/kobuki',
        description='Port série du Kobuki'
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
                'serial_port': LaunchConfiguration('kobuki_port')
            }.items()
        ),

        # 2- Robot description (URDF + TF) (Hors PushRosNamespace pour éviter le double prefix)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kobuki_description'),
                    'launch',
                    'coflot-robot_description_with_namespace.launch.py'
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

        # 6- Robot pose node
        Node(
            package='kobuki_pose',
            executable='kobuki_pose',
            name='robot_pose_node',
            remappings=remappings,
            output='screen'
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
            'params_file': LaunchConfiguration('params'),
            'use_namespace': 'True', # Indique au launchfile Nav2 d'utiliser le namespace
            'namespace': robot_name 
        }.items()
    )


    return LaunchDescription([
        # Déclaration des arguments
        robot_name_arg,
        map_file_arg,
        params_nav2_arg,
        use_sim_time_arg,
        lidar_port_arg,
        kobuki_port_arg,

        # Lancement du groupe de nœuds namespacés
        robot_namespaced_nodes,
        
        # Lancement des nœuds Nav2
        nav2_launch,
    ])
