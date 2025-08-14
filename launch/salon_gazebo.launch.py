#!/usr/bin/env python3
# launch/salon_gazebo.launch.py
# este es el de prueba, sus .launch.py no existen
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios de paquetes
    tutorial_pkg_dir = get_package_share_directory('tutorial_pkg')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
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

    # Archivo del mundo personalizado
    world_file_name = os.path.join(tutorial_pkg_dir, 'worlds', 'salon_world.world')
    
    # Comando para lanzar Gazebo con el mundo personalizado
    gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_name, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Incluir el launch estándar de TurtleBot3 (para robot_state_publisher y spawn)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn del robot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])