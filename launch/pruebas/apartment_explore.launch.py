# Archivo: ~/ros2_ws/src/tutorial_pkg/launch/apartment_explore.launch.py

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
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'apartment_explore.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_apartment.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'apartment_world.world']),
        description='Full path to world file to load'
    )

    # Reescribir parámetros para incluir use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 1. Lanzar Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # 2. Robot State Publisher - INMEDIATO
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Spawn del robot - DELAY 3s
    spawn_turtlebot_cmd = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3_waffle',
                           '-file', PathJoinSubstitution([turtlebot3_gazebo_dir, 'models', 
                                                         'turtlebot3_waffle', 'model.sdf']),
                           '-x', '1.0',
                           '-y', '0.8',
                           '-z', '0.01',
                           '-Y', '0.0'],
                output='screen'
            )
        ]
    )

    # 4. SLAM Toolbox - DELAY 6s
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

    # 5. Navigation2 - DELAY 10s
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

    # 6. Explore Lite - DELAY 15s
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

    # 7. RViz - DELAY 8s
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore.rviz'])
    
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

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        gazebo_launch,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        slam_launch,
        nav2_launch,
        explore_launch,
        rviz_cmd,
    ])