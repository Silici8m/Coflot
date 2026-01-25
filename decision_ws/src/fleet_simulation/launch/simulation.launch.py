import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. CONFIGURATION ---
    # On récupère le chemin du package de simulation où se trouve la map
    sim_pkg_share = get_package_share_directory('fleet_simulation')
    
    # Chemin par défaut vers la map
    default_map_path = os.path.join(sim_pkg_share, 'maps', 'map.yaml')
    
    # Argument pour changer la map si besoin en ligne de commande
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Chemin complet vers le fichier map.yaml'
    )

    # --- 2. NOEUDS NAV2 (Gestion de la Map) ---
    
    
    # Map Server : Charge le fichier map.pgm/yaml
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    # Lifecycle Manager : Active le map_server automatiquement
    # Sans lui, le map_server reste en état "Unconfigured"
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # --- 3. NOEUDS DE SIMULATION ---

    # Mock Fleet : Simule les robots (Positions, Batteries, Actions Nav2)
    # Vient du package fleet_simulation
    mock_fleet_node = Node(
        package='fleet_simulation',
        executable='mock_fleet',
        name='mock_fleet',
        output='screen'
    )

    # --- 4. CŒUR DÉCISIONNEL ---

    # Mission Manager : Le cerveau qui alloue les missions
    # Vient du package mission_manager
    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            {'allocation_strategy': 'smart_utility'} # On force la stratégie intelligente
        ]
    )

    # --- 5. ASSEMBLAGE ---
    return LaunchDescription([
        map_yaml_arg,
        map_server_node,
        lifecycle_node,
        mock_fleet_node,
        # mission_manager_node
    ])