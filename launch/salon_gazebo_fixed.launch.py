#!/usr/bin/env python3

# launch/salon_gazebo_fixed.launch.py
# Lanzador robusto para Gazebo con mundo del salón

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Directorios de paquetes
    tutorial_pkg_dir = get_package_share_directory('tutorial_pkg')
    
    # Verificar si turtlebot3_gazebo está disponible
    try:
        turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
        turtlebot3_available = True
    except:
        turtlebot3_available = False
        print("Warning: turtlebot3_gazebo package not found")
    
    try:
        turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
        description_available = True
    except:
        description_available = False
        print("Warning: turtlebot3_description package not found")
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0',
        description='Specify X position of robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Specify Y position of robot'
    )
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', 
        default_value='0.01',
        description='Specify Z position of robot'
    )

    # Verificar archivo del mundo
    world_file_name = os.path.join(tutorial_pkg_dir, 'worlds', 'salon_world.world')
    if not os.path.exists(world_file_name):
        print(f"ERROR: World file not found: {world_file_name}")
        # Fallback a mundo vacío
        world_file_name = 'worlds/empty.world'

    # =================================================================
    # COMANDO PARA LANZAR GAZEBO CON EL MUNDO PERSONALIZADO
    # =================================================================
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file_name,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        name='gazebo_server'
    )

    # =================================================================
    # ROBOT STATE PUBLISHER
    # =================================================================
    robot_state_publisher_node = None
    
    if description_available:
        # Buscar archivo URDF
        urdf_file_paths = [
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf'),
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf'),
        ]
        
        robot_description = None
        for urdf_path in urdf_file_paths:
            if os.path.exists(urdf_path):
                try:
                    with open(urdf_path, 'r') as infp:
                        robot_description = infp.read()
                    break
                except Exception as e:
                    print(f"Error reading URDF file {urdf_path}: {e}")
        
        if robot_description:
            robot_state_publisher_node = TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='robot_state_publisher',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_description
                        }]
                    )
                ]
            )
        else:
            print("Warning: Could not load URDF file")

    # =================================================================
    # SPAWN DEL ROBOT (MÉTODO ROBUSTO)
    # =================================================================
    
    # Intentar encontrar el modelo SDF
    spawn_robot_node = None
    
    if turtlebot3_available:
        # Rutas posibles para el modelo SDF
        model_sdf_paths = [
            os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
            os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
        ]
        
        model_sdf_path = None
        for path in model_sdf_paths:
            if os.path.exists(path):
                model_sdf_path = path
                break
        
        if model_sdf_path:
            # Método 1: usando spawn_entity.py directamente
            spawn_robot_node = TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_turtlebot3',
                        arguments=[
                            '-entity', 'turtlebot3',
                            '-file', model_sdf_path,
                            '-x', x_pose,
                            '-y', y_pose,
                            '-z', z_pose,
                            '-robot_namespace', '/'
                        ],
                        output='screen'
                    )
                ]
            )
        elif robot_description:
            # Método 2: usando robot_description topic
            spawn_robot_node = TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_turtlebot3',
                        arguments=[
                            '-entity', 'turtlebot3',
                            '-topic', '/robot_description',
                            '-x', x_pose,
                            '-y', y_pose,
                            '-z', z_pose
                        ],
                        output='screen'
                    )
                ]
            )
    
    # =================================================================
    # JOINT STATE PUBLISHER (PARA EVITAR PROBLEMAS DE TF)
    # =================================================================
    joint_state_publisher_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # =================================================================
    # CONSTRUIR LISTA DE ACCIONES
    # =================================================================
    launch_actions = [
        declare_use_sim_time_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        gazebo_cmd,
        joint_state_publisher_node
    ]
    
    # Agregar robot state publisher si está disponible
    if robot_state_publisher_node:
        launch_actions.append(robot_state_publisher_node)
    
    # Agregar spawn del robot si está disponible
    if spawn_robot_node:
        launch_actions.append(spawn_robot_node)
    else:
        print("Warning: Could not configure robot spawning")
    
    return LaunchDescription(launch_actions)