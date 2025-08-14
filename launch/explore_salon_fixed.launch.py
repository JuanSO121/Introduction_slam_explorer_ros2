#!/usr/bin/env python3
"""
Launch file corregido para exploración del salón
Combina el enfoque exitoso del proyecto de prueba con la funcionalidad completa
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # =================================================================
    # DIRECTORIOS Y CONFIGURACIONES
    # =================================================================
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
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
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0',
        description='specify x position of robot'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='specify y position of robot'
    )

    # =================================================================
    # CONFIGURAR PARÁMETROS
    # =================================================================
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # =================================================================
    # VARIABLES DE ENTORNO PARA GAZEBO
    # =================================================================
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [
            os.environ.get('GAZEBO_MODEL_PATH', ''),
            ':',
            os.path.join(tutorial_dir, 'models'),
            ':',
            os.path.join(turtlebot3_gazebo_dir, 'models')
        ]
    )

    # =================================================================
    # GAZEBO CON MUNDO DEL SALÓN (MÉTODO DIRECTO QUE FUNCIONA)
    # =================================================================
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen'
            )
        ]
    )

    # =================================================================
    # ROBOT STATE PUBLISHER
    # =================================================================
    robot_state_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': os.path.join(
                        turtlebot3_gazebo_dir, 
                        'urdf', 
                        'turtlebot3_waffle.urdf'
                    )
                }]
            )
        ]
    )

    # =================================================================
    # SPAWN DEL ROBOT (MÉTODO DIRECTO)
    # =================================================================
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-file', os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01',
                    '-timeout', '60.0'
                ],
                output='screen'
            )
        ]
    )

    # =================================================================
    # SLAM TOOLBOX (USANDO ARCHIVOS QUE SÍ EXISTEN)
    # =================================================================
    slam_toolbox = TimerAction(
        period=6.0,
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
    # NAVIGATION2 (MÉTODO SIMPLIFICADO)
    # =================================================================
    nav2_nodes = [
        TimerAction(
            period=8.0,
            actions=[
                # Costmap Global
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
                
                # AMCL
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[configured_params]
                ),
                
                # Lifecycle Manager
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl']
                    }]
                )
            ]
        ),
        
        TimerAction(
            period=10.0,
            actions=[
                # Planner Server
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[configured_params]
                ),
                
                # Controller Server
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[configured_params]
                ),
                
                # BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[configured_params]
                )
            ]
        )
    ]

    # =================================================================
    # EXPLORE LITE (NODO DIRECTO)
    # =================================================================
    explore_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[configured_params],
                remappings=[
                    ('/explore/frontiers', '/explore_frontiers'),
                    ('/explore/goal', '/explore_goal')
                ]
            )
        ]
    )

    # =================================================================
    # RVIZ
    # =================================================================
    rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'explore_robust.rviz')
    
    rviz_cmd = TimerAction(
        period=7.0,
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
    # NODOS DE MONITOREO (SI EXISTEN)
    # =================================================================
    monitoring_nodes = []
    
    # Solo agregar si los ejecutables existen
    try:
        exploration_monitor = TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='exploration_monitor',
                    name='exploration_monitor',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        monitoring_nodes.append(exploration_monitor)
    except:
        print("⚠️ exploration_monitor no disponible")

    try:
        exploration_visualizer = TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='exploration_visualizer',
                    name='exploration_visualizer',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        monitoring_nodes.append(exploration_visualizer)
    except:
        print("⚠️ exploration_visualizer no disponible")

    # =================================================================
    # DESCRIPCIÓN FINAL DEL LANZAMIENTO
    # =================================================================
    launch_description = [
        # Variables de entorno
        set_gazebo_model_path,
        
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        
        # Secuencia de lanzamiento
        gazebo_server,          # t=0: Gazebo server
        gazebo_client,          # t=2: Gazebo client  
        robot_state_publisher,  # t=3: Robot description
        spawn_robot,           # t=4: Spawn robot
        slam_toolbox,          # t=6: SLAM
        rviz_cmd,             # t=7: RViz
    ]
    
    # Agregar nodos de navegación
    launch_description.extend(nav2_nodes)
    
    # Agregar exploración
    launch_description.append(explore_node)  # t=12
    
    # Agregar nodos de monitoreo si están disponibles
    launch_description.extend(monitoring_nodes)

    return LaunchDescription(launch_description)