#!/usr/bin/env python3
"""
Launch file corregido para sistema de control por voz IA en tutorial_pkg
Ubicación: ~/ros2_ws/src/tutorial_pkg/launch/voice_control_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Obtener directorio del paquete
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Archivo de configuración
    voice_config_file = os.path.join(tutorial_dir, 'config', 'voice_control.yaml')
    
    # Verificar que existe el archivo
    if not os.path.exists(voice_config_file):
        print(f"⚠️  Archivo de configuración no encontrado: {voice_config_file}")
        print("💡 Creando configuración básica...")
        # Crear configuración mínima si no existe
        voice_config_file = None
    
    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_enable_ai_cmd = DeclareLaunchArgument(
        'enable_ai',
        default_value='true',
        description='Enable AI voice responses'
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'voice_config_file',
        default_value=voice_config_file if voice_config_file else '',
        description='Full path to voice control configuration file'
    )
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ai = LaunchConfiguration('enable_ai')
    config_file = LaunchConfiguration('voice_config_file')
    
    # === NODO PRINCIPAL: AI VOICE COMMANDER ===
    ai_voice_commander_params = [
        {'use_sim_time': use_sim_time, 'enable_ai': enable_ai}
    ]
    
    # Agregar archivo de configuración solo si existe
    if voice_config_file:
        ai_voice_commander_params.insert(0, config_file)
    
    ai_voice_commander_node = Node(
        package='tutorial_pkg',
        executable='ai_voice_commander',
        name='ai_voice_commander',
        output='screen',
        parameters=ai_voice_commander_params,
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/voice_commands', '/voice_commands'),
            ('/voice_feedback', '/voice_feedback'),
            ('/ai_context', '/ai_context'),
            ('/exploration_control', '/exploration_control')
        ]
    )
    
    # === NODO DE RESPUESTA IA ===
    ai_response_params = [
        {'use_sim_time': use_sim_time}
    ]
    
    # Agregar archivo de configuración solo si existe
    if voice_config_file:
        ai_response_params.insert(0, config_file)
    
    ai_response_node = TimerAction(
        period=2.0,  # Iniciar 2 segundos después
        actions=[
            Node(
                package='tutorial_pkg',
                executable='ai_response_node',
                name='ai_response_node',
                output='screen',
                parameters=ai_response_params,
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
    
    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_enable_ai_cmd,
        declare_config_file_cmd,
        
        # Nodos
        ai_voice_commander_node,
        ai_response_node
    ])