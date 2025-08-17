#!/usr/bin/env python3

# launch/explore_robust.launch.py
# Archivo corregido - Exploración robusta con carga optimizada

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    # =================================================================
    # CONFIGURACIÓN DE PAQUETES
    # =================================================================
    tutorial_dir = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # =================================================================
    # ARGUMENTOS DE LANZAMIENTO
    # =================================================================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore_robust.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_salon.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([tutorial_dir, 'rviz', 'salon_explore.rviz']),
        description='Full path to the RVIZ config file'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'salon_world.world']),
        description='Full path to the Gazebo world file'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position for robot spawn'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position for robot spawn'
    )
    
    # =================================================================
    # CONFIGURACIÓN DINÁMICA DE PARÁMETROS
    # =================================================================
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    # =================================================================
    # GAZEBO + ROBOT (USANDO EL LAUNCH CORREGIDO)
    # =================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([tutorial_dir, 'launch', 'salon_gazebo.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )
    
    # =================================================================
    # SLAM TOOLBOX OPTIMIZADO
    # =================================================================
    slam_launch = TimerAction(
        period=5.0,  # Esperar a que el robot esté spawneado
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': configured_slam_params
                }.items()
            )
        ]
    )
    
    # =================================================================
    # NAVIGATION2 STACK
    # =================================================================
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params
                }.items()
            )
        ]
    )
    
    # =================================================================
    # RVIZ PARA VISUALIZACIÓN
    # =================================================================
    rviz_launch = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # =================================================================
    # EXPLORE_LITE PARA EXPLORACIÓN AUTÓNOMA
    # =================================================================
    explore_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[
                    configured_params,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ]
            )
        ]
    )
    
    # =================================================================
    # NODOS DE MONITOREO Y DIAGNÓSTICO (OPCIONALES)
    # =================================================================
    exploration_monitor = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor.py',
                name='exploration_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=lambda context: os.path.exists(
                    os.path.join(
                        get_package_share_directory('tutorial_pkg'),
                        'scripts',
                        'exploration_monitor.py'
                    )
                )
            )
        ]
    )
    
    # =================================================================
    # HELPER NODES (CONDICIONALES)
    # =================================================================
    helper_nodes = GroupAction([
        # Limpiador de costmap (si existe)
        Node(
            package='tutorial_pkg',
            executable='costmap_cleaner.py',
            name='costmap_cleaner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ) if os.path.exists(
            os.path.join(
                get_package_share_directory('tutorial_pkg'),
                'scripts',
                'costmap_cleaner.py'
            )
        ) else Node(package='', executable=''),
        
        # Visualizador de exploración (si existe)
        Node(
            package='tutorial_pkg',
            executable='exploration_visualizer.py',
            name='exploration_visualizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ) if os.path.exists(
            os.path.join(
                get_package_share_directory('tutorial_pkg'),
                'scripts',
                'exploration_visualizer.py'
            )
        ) else Node(package='', executable='')
    ])
    
    helper_launch = TimerAction(
        period=18.0,
        actions=[helper_nodes]
    )
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_rviz_config_cmd,
        declare_world_file_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        
        # Lanzamiento secuencial optimizado
        gazebo_launch,        # t=0: Gazebo + Robot (usando salon_gazebo.launch.py)
        slam_launch,          # t=5: SLAM Toolbox
        rviz_launch,          # t=6: Visualización
        nav2_launch,          # t=8: Navigation Stack
        explore_launch,       # t=12: Exploración autónoma
        exploration_monitor,  # t=15: Monitor (si existe)
        helper_launch,        # t=18: Nodos auxiliares (si existen)
    ])


def get_package_share_directory(package_name):
    """Helper function para obtener directorio del paquete"""
    try:
        from ament_index_python.packages import get_package_share_directory as get_dir
        return get_dir(package_name)
    except Exception:
        return f"/opt/ros/humble/share/{package_name}"