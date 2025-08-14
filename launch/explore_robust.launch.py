#!/usr/bin/env python3
# launch/explore_robust.launch.py
# Archivo de lanzamiento corregido para exploración robusta

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                          ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directorios de paquetes
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world_file = LaunchConfiguration('world_file')

    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'explore_optimized.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'slam_optimized.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(tutorial_dir, 'worlds', 'salon_world.world'),
        description='Full path to the Gazebo world file'
    )

    # Reescribir parámetros para usar tiempo de simulación
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # =================================================================
    # GAZEBO CON MUNDO PERSONALIZADO (MÉTODO DIRECTO)
    # =================================================================
    
    # Usar ExecuteProcess para lanzar Gazebo directamente con nuestro mundo
    gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # =================================================================
    # ROBOT STATE PUBLISHER
    # =================================================================
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': os.path.join(
                turtlebot3_gazebo_dir, 
                'models', 
                'turtlebot3_waffle', 
                'model.sdf'
            )
        }]
    )

    # =================================================================
    # SPAWN ROBOT CON DELAY
    # =================================================================
    spawn_turtlebot_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-entity', 'waffle',
                    '-file', os.path.join(
                        turtlebot3_gazebo_dir,
                        'models',
                        'turtlebot3_waffle',
                        'model.sdf'
                    ),
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # =================================================================
    # SLAM TOOLBOX DIRECTO
    # =================================================================
    slam_cmd = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # =================================================================
    # RVIZ CON DELAY
    # =================================================================
    rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'explore_robust.rviz')
    
    rviz_cmd = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # =================================================================
    # NAVIGATION2 STACK
    # =================================================================
    nav2_cmd = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params
                }.items()
            )
        ]
    )

    # =================================================================
    # EXPLORE LITE DIRECTO
    # =================================================================
    explore_cmd = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[
                    configured_params,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # =================================================================
    # NODOS DE MONITOREO (OPCIONAL)
    # =================================================================
    exploration_monitor_cmd = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor.py',
                name='exploration_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        
        # Secuencia de lanzamiento
        gazebo_server_cmd,        # t=0: Gazebo server
        gazebo_client_cmd,        # t=0: Gazebo client  
        robot_state_publisher_cmd, # t=0: Robot description
        spawn_turtlebot_cmd,      # t=5: Spawn robot
        slam_cmd,                 # t=8: SLAM
        rviz_cmd,                # t=10: Visualización
        nav2_cmd,                # t=12: Navigation stack
        explore_cmd,             # t=15: Exploración
        exploration_monitor_cmd,  # t=18: Monitor (opcional)
    ])