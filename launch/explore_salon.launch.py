# launch/explore_salon.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
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
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo')

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
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_turtlebot.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'salon_world.world']),
        description='Full path to the Gazebo world file'
    )

    # Reescribir parámetros para incluir use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # =================================================================
    # LANZAR GAZEBO CON MUNDO PERSONALIZADO
    # =================================================================
    
    # Usar el lanzador completo de TurtleBot3 pero con nuestro mundo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawner del robot TurtleBot3 - con delay para esperar que Gazebo esté listo
    robot_spawner = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py'])
                ]),
                launch_arguments={
                    'x_pose': '0.0',
                    'y_pose': '0.0',
                    'z_pose': '0.01',
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # =================================================================
    # SLAM TOOLBOX - con delay
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
    # NAVIGATION2 - con delay para esperar SLAM
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
    # EXPLORE LITE - con delay para esperar navegación
    # =================================================================
    explore_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([explore_lite_dir, 'launch', 'explore.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # =================================================================
    # RVIZ CON CONFIGURACIÓN PARA EXPLORACIÓN
    # =================================================================
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore.rviz'])
    
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

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        
        # Lanzar en secuencia con delays apropiados
        gazebo_launch,          # t=0: Gazebo + mundo
        robot_spawner,          # t=3: Spawn robot
        slam_launch,            # t=5: SLAM
        rviz_cmd,              # t=6: RViz
        nav2_launch,           # t=8: Navigation
        explore_launch,        # t=12: Exploration
    ])