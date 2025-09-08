#!/usr/bin/env python3
"""
Launch file maestro integrado para tutorial_pkg - VERSIÓN CORREGIDA
Combina exploración autónoma con control por voz IA
Ubicación: ~/ros2_ws/src/tutorial_pkg/launch/integrated_exploration_launch_fixed.py
"""

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import subprocess


def launch_setup(context, *args, **kwargs):
    """Configuración dinámica del sistema integrado - MÉTODO ROBUSTO"""
    
    # Directorios
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    enable_voice_control = LaunchConfiguration('enable_voice_control')
    enable_flask_server = LaunchConfiguration('enable_flask_server')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    actions = []
    
    # =================================================================
    # 1. GAZEBO CON MUNDO PERSONALIZADO (MÉTODO ROBUSTO)
    # =================================================================
    world_path = os.path.join(tutorial_dir, 'worlds', 'salon_world.world')
    
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        name='gazebo_server'
    )
    actions.append(gazebo_cmd)
    
    # =================================================================
    # 2. ROBOT STATE PUBLISHER (MÉTODO DIRECTO Y ROBUSTO)
    # =================================================================
    
    # Encontrar el archivo URDF del TurtleBot3 - MÉTODO CORRECTO
    urdf_file = None
    robot_description = None
    
    # Lista de rutas posibles para el URDF (en orden de preferencia)
    urdf_paths_to_try = []
    
    try:
        turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
        urdf_paths_to_try.extend([
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf'),
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
        ])
    except:
        pass
    
    # Buscar archivo URDF válido
    for urdf_path in urdf_paths_to_try:
        if os.path.exists(urdf_path):
            try:
                with open(urdf_path, 'r') as infp:
                    robot_description_raw = infp.read()
                    # Limpiar variables no resueltas
                    robot_description = robot_description_raw.replace('${namespace}', '')
                urdf_file = urdf_path
                print(f"✅ URDF encontrado: {urdf_path}")
                break
            except Exception as e:
                print(f"❌ Error reading URDF {urdf_path}: {e}")
                continue
    
    if robot_description:
        robot_state_publisher_cmd = TimerAction(
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
        actions.append(robot_state_publisher_cmd)
        print("✅ Robot State Publisher configurado")
    else:
        print("❌ No se pudo cargar el URDF del robot")
    
    # =================================================================
    # 3. SPAWN ROBOT (MÉTODO ROBUSTO)
    # =================================================================
    
    # Primero intentar con archivos SDF
    try:
        turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
        sdf_paths = [
            os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
            os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
        ]
        
        sdf_file_found = None
        for sdf_path in sdf_paths:
            if os.path.exists(sdf_path):
                sdf_file_found = sdf_path
                break
        
        if sdf_file_found:
            spawn_robot_cmd = TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_turtlebot3',
                        arguments=[
                            '-entity', 'turtlebot3',
                            '-file', sdf_file_found,
                            '-x', context.perform_substitution(x_pose),
                            '-y', context.perform_substitution(y_pose),
                            '-z', context.perform_substitution(z_pose),
                            '-robot_namespace', ''
                        ],
                        output='screen'
                    )
                ]
            )
            actions.append(spawn_robot_cmd)
            print(f"✅ Spawn robot configurado con SDF: {sdf_file_found}")
        else:
            # Fallback: usar topic de robot_description
            spawn_robot_cmd = TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_turtlebot3',
                        arguments=[
                            '-entity', 'turtlebot3',
                            '-topic', '/robot_description',
                            '-x', context.perform_substitution(x_pose),
                            '-y', context.perform_substitution(y_pose),
                            '-z', context.perform_substitution(z_pose),
                            '-robot_namespace', ''
                        ],
                        output='screen'
                    )
                ]
            )
            actions.append(spawn_robot_cmd)
            print("✅ Spawn robot configurado con topic")
    except Exception as e:
        print(f"❌ Error configurando spawn: {e}")
    
    # =================================================================
    # 4. SLAM TOOLBOX (MÉTODO ROBUSTO)
    # =================================================================
    try:
        slam_toolbox_dir = get_package_share_directory('slam_toolbox')
        slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        
        if os.path.exists(slam_launch_file):
            slam_launch = TimerAction(
                period=6.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch_file),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'slam_params_file': slam_params_file
                        }.items()
                    )
                ]
            )
            actions.append(slam_launch)
            print("✅ SLAM Toolbox configurado con launch file")
        else:
            # SLAM directo con nodo
            slam_direct = TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package='slam_toolbox',
                        executable='async_slam_toolbox_node',
                        name='slam_toolbox',
                        output='screen',
                        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
                    )
                ]
            )
            actions.append(slam_direct)
            print("✅ SLAM Toolbox configurado directamente")
    except Exception as e:
        print(f"❌ Warning: SLAM Toolbox not available: {e}")
    
    # =================================================================
    # 5. NAVIGATION2 (MÉTODO ROBUSTO)
    # =================================================================
    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        
        if os.path.exists(nav_launch_file):
            nav2_launch = TimerAction(
                period=9.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch_file),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'params_file': params_file
                        }.items()
                    )
                ]
            )
            actions.append(nav2_launch)
            print("✅ Navigation2 configurado")
        else:
            print("❌ Nav2 navigation launch file not found")
    except Exception as e:
        print(f"❌ Warning: Nav2 not available: {e}")
    
    # =================================================================
    # 6. MONITOR DE EXPLORACIÓN AVANZADO
    # =================================================================
    exploration_monitor = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor',
                name='exploration_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_growth_timeout': 45.0,
                    'position_stuck_timeout': 30.0,
                    'min_frontier_distance': 1.5,
                    'max_exploration_distance': 8.0
                }],
                remappings=[
                    ('/map', '/map'),
                    ('/odom', '/odom'),
                    ('/scan', '/scan'),
                    ('/cmd_vel', '/cmd_vel'),
                    ('/goal_pose', '/goal_pose')
                ]
            )
        ]
    )
    actions.append(exploration_monitor)
    print("✅ Exploration Monitor configurado")
    
    # =================================================================
    # 7. SISTEMA DE CONTROL POR VOZ IA (CONDICIONAL)
    # =================================================================
    
    # AI Voice Commander
    voice_commander_group = GroupAction(
        condition=IfCondition(enable_voice_control),
        actions=[
            TimerAction(
                period=13.0,
                actions=[
                    Node(
                        package='tutorial_pkg',
                        executable='ai_voice_commander',
                        name='ai_voice_commander',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'enable_ai_responses': True,
                            'linear_speed_default': 0.2,
                            'angular_speed_default': 0.3,
                            'auto_stop_timeout': 5.0,
                            'emergency_stop_enabled': True
                        }],
                        remappings=[
                            ('/cmd_vel', '/cmd_vel'),
                            ('/voice_commands', '/voice_commands'),
                            ('/voice_feedback', '/voice_feedback'),
                            ('/ai_context', '/ai_context'),
                            ('/exploration_control', '/exploration_control')
                        ]
                    )
                ]
            )
        ]
    )
    actions.append(voice_commander_group)
    
    # AI Response Node
    ai_response_group = GroupAction(
        condition=IfCondition(enable_voice_control),
        actions=[
            TimerAction(
                period=15.0,
                actions=[
                    Node(
                        package='tutorial_pkg',
                        executable='ai_response_node',
                        name='ai_response_node',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'status_publish_rate': 2.0,
                            'heartbeat_interval': 30.0
                        }],
                        remappings=[
                            ('/voice_feedback', '/voice_feedback'),
                            ('/cmd_vel', '/cmd_vel'),
                            ('/map', '/map'),
                            ('/ai_status', '/ai_status'),
                            ('/robot_state', '/robot_state'),
                            ('/exploration_control', '/exploration_control')
                        ]
                    )
                ]
            )
        ]
    )
    actions.append(ai_response_group)
    
    # =================================================================
    # 8. EXPLORE LITE (MÉTODO ROBUSTO)
    # =================================================================
    try:
        explore_lite_dir = get_package_share_directory('explore_lite')
        explore_launch_file = os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
        
        if os.path.exists(explore_launch_file):
            explore_launch = TimerAction(
                period=17.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(explore_launch_file),
                        launch_arguments={
                            'use_sim_time': use_sim_time
                        }.items()
                    )
                ]
            )
            actions.append(explore_launch)
            print("✅ Explore Lite configurado con launch file")
        else:
            # Exploración directa con nodo
            explore_direct = TimerAction(
                period=17.0,
                actions=[
                    Node(
                        package='explore_lite',
                        executable='explore',
                        name='explore_node',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_base_frame': 'base_footprint',
                            'costmap_topic': '/global_costmap/costmap',
                            'costmap_updates_topic': '/global_costmap/costmap_updates',
                            'visualize': True,
                            'planner_frequency': 0.33,
                            'progress_timeout': 30.0,
                            'potential_scale': 3.0,
                            'orientation_scale': 0.0,
                            'gain_scale': 1.0,
                            'transform_tolerance': 1.0,
                            'min_frontier_size': 0.5,
                            'frontier_travel_point': 'centroid'
                        }]
                    )
                ]
            )
            actions.append(explore_direct)
            print("✅ Explore Lite configurado directamente")
    except Exception as e:
        print(f"❌ Warning: Explore Lite not available: {e}")
    
    # =================================================================
    # 9. RVIZ CON CONFIGURACIÓN PERSONALIZADA
    # =================================================================
    rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'integrated_exploration.rviz')
    
    # Si no existe el archivo personalizado, usar el básico
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'explore.rviz')
    
    if os.path.exists(rviz_config_file):
        rviz_cmd = TimerAction(
            period=10.0,
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
        actions.append(rviz_cmd)
        print(f"✅ RViz configurado: {rviz_config_file}")
    
    # =================================================================
    # 10. SERVIDOR FLASK (CONDICIONAL)
    # =================================================================
    flask_server_group = GroupAction(
        condition=IfCondition(enable_flask_server),
        actions=[
            TimerAction(
                period=20.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'python3',
                            os.path.join(tutorial_dir, 'voice_services', 'flask_server.py')
                        ],
                        output='screen',
                        name='flask_voice_server',
                        cwd=os.path.join(tutorial_dir, 'voice_services')
                    )
                ]
            )
        ]
    )
    actions.append(flask_server_group)
    
    # =================================================================
    # 11. NODOS DE MONITOREO Y VISUALIZACIÓN (VERIFICACIÓN DE EXISTENCIA)
    # =================================================================
    
    # Visualizador de exploración
    visualizer_script = os.path.join(tutorial_dir, 'scripts', 'exploration_visualizer.py')
    if os.path.exists(visualizer_script):
        exploration_visualizer = TimerAction(
            period=12.0,
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
        actions.append(exploration_visualizer)
        print("✅ Exploration Visualizer configurado")
    
    # Reiniciador de exploración
    restarter_script = os.path.join(tutorial_dir, 'scripts', 'exploration_restarter.py')
    if os.path.exists(restarter_script):
        exploration_restarter = TimerAction(
            period=25.0,
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
        actions.append(exploration_restarter)
        print("✅ Exploration Restarter configurado")
    
    print(f"🚀 Sistema integrado configurado con {len(actions)} componentes")
    return actions


def generate_launch_description():
    """Generar descripción de lanzamiento completa - VERSIÓN CORREGIDA"""
    
    # Directorio del paquete
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # =================================================================
    # ARGUMENTOS DE LANZAMIENTO
    # =================================================================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(tutorial_dir, 'worlds', 'salon_world.world'),
        description='Full path to the Gazebo world file'
    )
    
    declare_enable_voice_control_cmd = DeclareLaunchArgument(
        'enable_voice_control',
        default_value='true',
        description='Enable AI voice control system'
    )
    
    declare_enable_flask_server_cmd = DeclareLaunchArgument(
        'enable_flask_server',
        default_value='false',
        description='Enable Flask server for mobile app integration'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0',
        description='Initial X position of robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Initial Y position of robot'
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', 
        default_value='0.01',
        description='Initial Z position of robot'
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='waffle',
        description='TurtleBot3 model (burger, waffle, waffle_pi)'
    )
    
    declare_map_save_path_cmd = DeclareLaunchArgument(
        'map_save_path',
        default_value=os.path.join(tutorial_dir, 'maps'),
        description='Directory to save generated maps'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'navigation_simple.yaml'),
        description='Full path to the ROS2 parameters file for navigation'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'slam_simple.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    return LaunchDescription([
        # Argumentos de configuración
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        declare_enable_voice_control_cmd,
        declare_enable_flask_server_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        declare_robot_model_cmd,
        declare_map_save_path_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        
        # Sistema integrado
        OpaqueFunction(function=launch_setup)
    ])