from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    # Directorios
    tutorial_dir = FindPackageShare('tutorial_pkg')
    aws_house_dir = FindPackageShare('aws_robomaker_small_house_world')
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    world_file = LaunchConfiguration('world_file')

    # Argumentos
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore.yaml']),
        description='Parameters file'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([aws_house_dir, 'worlds', 'small_house.world']),
        description='World file'
    )

    # Lanzar Gazebo con el mundo de la casa
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot description
    urdf_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file.perform(context=None)).read()
        }]
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_world_cmd,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
    ])
