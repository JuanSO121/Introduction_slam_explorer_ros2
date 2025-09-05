#!/usr/bin/env python3
"""
Launch file para sistema completo con control por voz
Ubicación: tutorial_pkg/launch/voice_control_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def launch_setup(context, *args, **kwargs):
    """Función de configuración dinámica del launch"""
    
    # Directorios de paquetes
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    voice_config_file = LaunchConfiguration('voice_config_file')
    enable_voice_control = LaunchConfiguration('enable_voice_control')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge')
    world_file = LaunchConfiguration('world_file')
    
    # Lista de acciones a retornar
    actions = []
    
    # =================================================================
    # INFORMACIÓN DE INICIO
    # =================================================================
    actions.append(LogInfo(msg="🤖🎤 Iniciando Sistema de Control por Voz"))
    actions.append(LogInfo(msg="==================================================="))
    
    # =================================================================
    # ROSBRIDGE SERVER (para comunicación con Windows)
    # =================================================================
    if context.perform_substitution(enable_rosbridge).lower() == 'true':
        rosbridge_server = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0'  # Permitir conexiones externas
            }],
            output='screen'
        )
        actions.append(rosbridge_server)
        actions.append(LogInfo(msg="🌉 ROSBridge iniciado en puerto 9090"))
    
    # =================================================================
    # INCLUIR LAUNCH PRINCIPAL DE EXPLORACIÓN
    # =================================================================
    explore_launch_file = os.path.join(tutorial_dir, 'launch', 'explore_robust_fixed.launch.py')
    
    if os.path.exists(explore_launch_file):
        explore_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'world_file': world_file
            }.items()
        )
        actions.append(explore_launch)
        actions.append(LogInfo(msg="🗺️ Sistema de exploración cargado"))
    else:
        actions.append(LogInfo(msg="⚠️ Archivo de exploración no encontrado, continuando sin él"))
    
    # =================================================================
    # NODO MANEJADOR DE COMANDOS DE VOZ
    # =================================================================
    if context.perform_substitution(enable_voice_control).lower() == 'true':
        
        voice_command_handler = TimerAction(
            period=5.0,  # Esperar 5s para que otros servicios inicien
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='voice_command_handler',
                    name='voice_command_handler',
                    output='screen',
                    parameters=[voice_config_file, {'use_sim_time': use_sim_time}],
                    remappings=[
                        # Remapear tópicos si es necesario
                        ('/cmd_vel', '/cmd_vel'),
                        ('/goal_pose', '/goal_pose'),
                    ]
                )
            ]
        )
        actions.append(voice_command_handler)
        actions.append(LogInfo(msg="🎙️ Manejador de comandos de voz habilitado"))
    
    # =================================================================
    # NODO MONITOR AVANZADO DE EXPLORACIÓN (con integración de voz)
    # =================================================================
    enhanced_monitor = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor',  # Usar el monitor existente
                name='voice_enhanced_exploration_monitor',
                output='screen',
                parameters=[voice_config_file, {'use_sim_time': use_sim_time}]
            )
        ]
    )
    actions.append(enhanced_monitor)
    
    # =================================================================
    # NODO DE ESTADO DE VOZ (para reportar estado al servidor Windows)
    # =================================================================
    if context.perform_substitution(enable_voice_control).lower() == 'true':
        
        voice_status_reporter = TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='voice_status_reporter.py',
                    name='voice_status_reporter',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        # Comentado porque el script no existe aún - se puede agregar después
        # actions.append(voice_status_reporter)
    
    # =================================================================
    # RVIZ CON CONFIGURACIÓN DE VOZ
    # =================================================================
    rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'voice_control.rviz')
    if not os.path.exists(rviz_config_file):
        # Fallback a configuración de exploración normal
        rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'explore.rviz')
    
    if os.path.exists(rviz_config_file):
        rviz_cmd = TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_voice_control',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('launch_rviz'))
                )
            ]
        )
        actions.append(rviz_cmd)
    
    # =================================================================
    # NODOS DE DIAGNÓSTICO Y MONITOREO
    # =================================================================
    
    # Monitor de conectividad ROSBridge
    if context.perform_substitution(enable_rosbridge).lower() == 'true':
        connectivity_monitor = TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='connectivity_monitor.py',
                    name='rosbridge_connectivity_monitor',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'rosbridge_port': 9090,
                        'check_interval': 30.0
                    }]
                )
            ]
        )
        # Comentado - script a crear si se necesita
        # actions.append(connectivity_monitor)
    
    # =================================================================
    # CONFIGURACIÓN FINAL
    # =================================================================
    actions.append(LogInfo(msg="🎉 Sistema de Control por Voz listo"))
    actions.append(LogInfo(msg="📡 Servidor Windows puede conectarse en puerto 9090"))
    actions.append(LogInfo(msg="🎙️ Comandos de voz listos para recibir"))
    actions.append(LogInfo(msg="==================================================="))
    
    return actions


def generate_launch_description():
    # Directorios de paquetes
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'navigation_simple.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_voice_config_file_cmd = DeclareLaunchArgument(
        'voice_config_file',
        default_value=os.path.join(tutorial_dir, 'config', 'voice_commands.yaml'),
        description='Full path to voice commands configuration file'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(tutorial_dir, 'worlds', 'salon_world.world'),
        description='Full path to the Gazebo world file'
    )
    
    declare_enable_voice_control_cmd = DeclareLaunchArgument(
        'enable_voice_control',
        default_value='true',
        description='Enable voice control system'
    )
    
    declare_enable_rosbridge_cmd = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='true',
        description='Enable ROSBridge for Windows communication'
    )
    
    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_voice_config_file_cmd,
        declare_world_file_cmd,
        declare_enable_voice_control_cmd,
        declare_enable_rosbridge_cmd,
        declare_launch_rviz_cmd,
        
        # Configuración dinámica
        OpaqueFunction(function=launch_setup)
    ])