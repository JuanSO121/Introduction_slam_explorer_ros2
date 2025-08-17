#!/usr/bin/env python3

# launch/salon_gazebo.launch.py
# Archivo corregido - Carga optimizada del mundo y robot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =================================================================
    # CONFIGURACIÓN DE PAQUETES Y RUTAS
    # =================================================================
    tutorial_pkg_dir = get_package_share_directory('tutorial_pkg')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    world_file = LaunchConfiguration('world_file')
    
    # =================================================================
    # ARGUMENTOS DE LANZAMIENTO
    # =================================================================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0',
        description='X position of robot spawn point'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Y position of robot spawn point'
    )
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', 
        default_value='0.01',
        description='Z position of robot spawn point'
    )
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', 
        default_value='0.0',
        description='Yaw orientation of robot'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_pkg_dir, 'worlds', 'salon_world.world']),
        description='Full path to world file to load'
    )
    
    # =================================================================
    # VERIFICACIÓN Y CONFIGURACIÓN DEL MODELO TURTLEBOT3
    # =================================================================
    def get_robot_description():
        """Función para obtener la descripción del robot"""
        urdf_file = os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            'turtlebot3_waffle.urdf'
        )
        
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
            
        return robot_description
    
    # =================================================================
    # GAZEBO CON MUNDO PERSONALIZADO
    # =================================================================
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen',
        name='gazebo_server'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        name='gazebo_client'
    )
    
    # =================================================================
    # ROBOT STATE PUBLISHER (DESCRIPCIÓN DEL ROBOT)
    # =================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': get_robot_description()
        }]
    )
    
    # =================================================================
    # JOINT STATE PUBLISHER (PARA ARTICULACIONES)
    # =================================================================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # =================================================================
    # SPAWN DEL ROBOT EN GAZEBO (CON DELAY)
    # =================================================================
    spawn_robot = TimerAction(
        period=3.0,  # Esperar 3 segundos para que Gazebo esté listo
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-Y', yaw
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # =================================================================
    # CONFIGURACIÓN DE TRANSFORMACIONES
    # =================================================================
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        declare_use_sim_time_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        declare_yaw_cmd,
        declare_world_file_cmd,
        
        # Lanzamiento secuencial
        gazebo_server,              # t=0: Servidor Gazebo
        gazebo_client,              # t=0: Cliente Gazebo
        robot_state_publisher,      # t=0: Descripción del robot
        joint_state_publisher,      # t=0: Estados de articulaciones
        static_transform_publisher, # t=0: Transformaciones estáticas
        spawn_robot,               # t=3: Spawn del robot con delay
    ])