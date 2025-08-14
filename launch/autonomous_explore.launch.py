#!/usr/bin/env python3
"""
Launch file para exploración completamente autónoma con auto-recuperación
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Directorios
    tutorial_dir = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    explore_lite_dir = FindPackageShare('explore_lite')
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo')

    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world_file = LaunchConfiguration('world_file')

    # Argumentos
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore_ultra.yaml']),
        description='Nav2 parameters file'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_turtlebot.yaml']),
        description='SLAM parameters file'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='turtlebot3_world.world',
        description='Gazebo world file'
    )

    # Configurar parámetros con use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 1. Gazebo con TurtleBot3
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # 3. Navigation2 con parámetros ultra-robustos
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params
        }.items()
    )

    # 4. Monitor de exploración autónomo (REEMPLAZA explore_lite)
    autonomous_monitor_node = Node(
        package='tutorial_pkg',
        executable='exploration_monitor',
        name='autonomous_exploration_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True,  # Auto-reiniciar si falla
        respawn_delay=5.0
    )

    # 5. RViz con configuración optimizada
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore.rviz'])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. Mapper inicial (opcional - con delay)
    initial_mapper_node = TimerAction(
        period=10.0,  # Esperar 10 segundos antes de iniciar
        actions=[
            Node(
                package='tutorial_pkg',
                executable='initial_mapper',
                name='initial_mapper',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 7. Limpiador de costmap de emergencia
    emergency_cleaner_node = Node(
        package='tutorial_pkg',
        executable='costmap_cleaner',
        name='emergency_costmap_cleaner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=3.0
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        
        # Lanzar en orden con delays
        gazebo_launch,
        
        TimerAction(
            period=5.0,
            actions=[slam_launch]
        ),
        
        TimerAction(
            period=8.0,
            actions=[nav2_launch]
        ),
        
        TimerAction(
            period=12.0,
            actions=[autonomous_monitor_node]
        ),
        
        TimerAction(
            period=15.0,
            actions=[emergency_cleaner_node]
        ),
        
        rviz_node,
        # initial_mapper_node,  # Descomentado si quieres mapper inicial
    ])