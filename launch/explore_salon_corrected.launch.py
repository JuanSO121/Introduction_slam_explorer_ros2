#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    # Directorios de paquetes
    tutorial_dir = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    explore_lite_dir = FindPackageShare('explore_lite')
    gazebo_ros_pkgs_dir = FindPackageShare('gazebo_ros')

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
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore_optimized.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_optimized.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'salon_world.world']),
        description='Full path to the Gazebo world file'
    )

    # Reescribir parámetros
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # =================================================================
    # CONFIGURAR GAZEBO MODEL PATH PARA INCLUIR NUESTRO MODELO
    # =================================================================
    models_path = PathJoinSubstitution([tutorial_dir, 'models'])
    
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # =================================================================
    # GAZEBO CON MUNDO PERSONALIZADO
    # =================================================================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkgs_dir, 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkgs_dir, 'launch', 'gzclient.launch.py'])
        ])
    )

    # =================================================================
    # ROBOT SPAWNER CON DELAY
    # =================================================================
    robot_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3_burger',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # =================================================================
    # ROBOT STATE PUBLISHER
    # =================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': os.popen('ros2 run xacro xacro $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro').read()
        }],
        output='screen'
    )

    # =================================================================
    # SLAM TOOLBOX CON CONFIGURACIÓN OPTIMIZADA
    # =================================================================
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file
                }.items()
            )
        ]
    )

    # =================================================================
    # NAVIGATION2 CON PARÁMETROS OPTIMIZADOS
    # =================================================================
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params
                }.items()
            )
        ]
    )

    # =================================================================
    # VISUALIZADOR DE EXPLORACIÓN
    # =================================================================
    exploration_visualizer = TimerAction(
        period=11.0,
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

    # =================================================================
    # MONITOR DE EXPLORACIÓN (NODO PERSONALIZADO)
    # =================================================================
    exploration_monitor = TimerAction(
        period=10.0,
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

    # =================================================================
    # EXPLORE LITE CON DELAY
    # =================================================================
    explore_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore',
                output='screen',
                parameters=[configured_params],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            )
        ]
    )

    # =================================================================
    # RVIZ OPTIMIZADO PARA EXPLORACIÓN
    # =================================================================
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore_robust.rviz'])
    
    rviz_cmd = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # =================================================================
    # NODO PARA REINICIAR EXPLORACIÓN SI SE DETIENE
    # =================================================================
    exploration_restarter = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_restarter',
                name='exploration_restarter',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        # Variables de entorno
        set_gazebo_model_path,
        
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        
        # Robot description
        robot_state_publisher,
        
        # Lanzar en secuencia
        gazebo_server,          # t=0: Gazebo server
        gazebo_client,          # t=0: Gazebo client
        robot_spawner,          # t=3: Spawn robot
        slam_launch,            # t=5: SLAM optimizado
        rviz_cmd,              # t=6: RViz
        nav2_launch,           # t=8: Navigation optimizada
        exploration_monitor,    # t=10: Monitor de exploración
        exploration_visualizer, # t=11: Visualizador de estado
        explore_launch,        # t=12: Exploración
        exploration_restarter,  # t=15: Reiniciador automático
    ])