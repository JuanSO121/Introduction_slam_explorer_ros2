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
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'salon_explore.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_salon.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'salon_world.world']),
        description='Full path to world file to load'
    )

    # Configurar la variable de entorno GAZEBO_MODEL_PATH
    # gazebo_model_path = PathJoinSubstitution([tutorial_dir, 'models'])

    # Configurar variables de entorno para Gazebo
    gazebo_env = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{PathJoinSubstitution([tutorial_dir, "models"])}'
        ],
        output='screen'
    )

    # Parámetros reescritos
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # Lanzar Gazebo con el mundo del salón
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
        '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world_file')],

        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': PathJoinSubstitution([tutorial_dir, 'models'])
        }
    )

    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn del robot en posición central del salón
    spawn_turtlebot_cmd = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3_waffle',
                           '-file', PathJoinSubstitution([turtlebot3_gazebo_dir, 'models', 
                                                         'turtlebot3_waffle', 'model.sdf']),
                           '-x', '0.0',  # Ajusta según el centro de tu salón
                           '-y', '0.0',
                           '-z', '0.01',
                           '-Y', '0.0'],
                output='screen'
            )
        ]
    )

    # SLAM
    slam_launch = TimerAction(
        period=6.0,
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

    # Navigation2
    nav2_launch = TimerAction(
        period=10.0,
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

    # Explore Lite
    explore_launch = TimerAction(
        period=15.0,
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

    # RViz
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'salon_explore.rviz'])
    
    rviz_cmd = TimerAction(
        period=8.0,
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
    
    # 8. Monitor de exploración - DELAY 20s
    exploration_monitor = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor',
                name='exploration_monitor',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        gazebo_env,
        gazebo_launch,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        slam_launch,
        nav2_launch,
        explore_launch,
        rviz_cmd,
        exploration_monitor,
    ])