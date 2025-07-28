from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    aws_house_dir = FindPackageShare('aws_robomaker_small_house_world')
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
        default_value=PathJoinSubstitution([aws_house_dir, 'worlds', 'small_house.world']),
        description='Full path to world file to load'
    )

    # Reescribir parámetros para incluir use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # Lanzar AWS Small House World con TurtleBot3
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                aws_house_dir,
                'launch',
                'small_house.launch.py'
            ])
        ]),
        launch_arguments={
            'gui': 'true',
            'server': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn TurtleBot3 en el mundo AWS
    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # Navigation2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params
        }.items()
    )

    # Explore Lite
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([explore_lite_dir, 'launch', 'explore.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # RViz con configuración para exploración
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore.rviz'])
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        gazebo_launch,
        robot_state_publisher,
        robot_spawn,
        slam_launch,
        nav2_launch,
        explore_launch,
        rviz_cmd,
    ])