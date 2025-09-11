#!/usr/bin/env python3
"""
Launch file para integrar Whisper Service con tutorial_pkg
Ubicación: ~/ros2_ws/src/tutorial_pkg/launch/whisper_integration_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    """Configurar integración Whisper manteniendo tutorial_pkg intacto"""
    
    # Directorios
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Configuraciones
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_whisper = LaunchConfiguration('enable_whisper')
    whisper_model = LaunchConfiguration('whisper_model')
    whisper_port = LaunchConfiguration('whisper_port')
    include_tutorial_system = LaunchConfiguration('include_tutorial_system')
    
    actions = []
    
    # =================================================================
    # 1. INCLUIR SISTEMA TUTORIAL_PKG EXISTENTE (OPCIONAL)
    # =================================================================
    
    # Si se desea, incluir el sistema completo de tutorial_pkg
    tutorial_main_launch = os.path.join(tutorial_dir, 'launch', 'integrated_exploration_launch.py')
    
    if os.path.exists(tutorial_main_launch):
        tutorial_system = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tutorial_main_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_voice_control': 'true',
                'enable_flask_server': 'false',  # No usar el Flask original
            }.items(),
            condition=IfCondition(include_tutorial_system)
        )
        actions.append(tutorial_system)
        print("✅ Sistema tutorial_pkg incluido")
    
    # =================================================================
    # 2. SERVICIO WHISPER FASTAPI
    # =================================================================
    
    whisper_service = TimerAction(
        period=3.0,  # Dar tiempo a que ROS2 se inicialice
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tutorial_pkg', 'whisper_fastapi_service'],
                output='screen',
                name='whisper_fastapi_service',
                additional_env={
                    'ROS_DOMAIN_ID': '0'
                },
                condition=IfCondition(enable_whisper)
            )
        ]
    )
    actions.append(whisper_service)
    
    # =================================================================
    # 3. BRIDGE NODE MEJORADO (OPCIONAL - para compatibilidad con Flutter existente)
    # =================================================================
    
    # Solo si quieres mantener compatibilidad con el sistema existente
    flutter_bridge_config = os.path.join(tutorial_dir, 'config', 'flutter_bridge_config.yaml')
    
    enhanced_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='flutter_bridge_node',
                name='flutter_bridge_enhanced',
                output='screen',
                parameters=[
                    flutter_bridge_config if os.path.exists(flutter_bridge_config) else {},
                    {
                        'use_sim_time': use_sim_time,
                        'whisper_integration': True,
                        'voice_commands_topic': '/voice_commands',
                        'transcription_feedback_topic': '/transcription_feedback'
                    }
                ],
                remappings=[
                    ('/voice_commands', '/voice_commands'),
                    ('/transcription_feedback', '/transcription_feedback'),
                    ('/robot_state', '/robot_state'),
                    ('/ai_status', '/ai_status')
                ]
            )
        ]
    )
    # Comentado por defecto para evitar duplicados con el sistema existente
    # actions.append(enhanced_bridge)
    
    # =================================================================
    # 4. MONITOR DE WHISPER (NODO DE DIAGNÓSTICO)
    # =================================================================
    
    whisper_monitor = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='whisper_monitor_node',
                name='whisper_service_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'whisper_service_url': f'http://localhost:{context.perform_substitution(whisper_port)}',
                    'health_check_interval': 10.0,
                    'reconnect_attempts': 5
                }],
                condition=IfCondition(enable_whisper)
            )
        ]
    )
    # Este nodo lo crearemos si es necesario
    # actions.append(whisper_monitor)
    
    port_value = context.perform_substitution(whisper_port)
    model_value = context.perform_substitution(whisper_model)
    
    print(f"🎤 Whisper Service configurado:")
    print(f"   - Puerto: {port_value}")
    print(f"   - Modelo: {model_value}")
    print(f"   - Endpoint: http://localhost:{port_value}/transcribe")
    print(f"   - Health: http://localhost:{port_value}/health")
    
    return actions

def generate_launch_description():
    """Generar descripción de lanzamiento para Whisper integration"""
    
    return LaunchDescription([
        # Argumentos de configuración
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'enable_whisper',
            default_value='true',
            description='Enable Whisper transcription service'
        ),
        
        DeclareLaunchArgument(
            'whisper_model',
            default_value='openai/whisper-small',
            description='Whisper model to use (small, base, large)',
            choices=['openai/whisper-tiny', 'openai/whisper-base', 'openai/whisper-small', 'openai/whisper-medium']
        ),
        
        DeclareLaunchArgument(
            'whisper_port',
            default_value='8000',
            description='Port for Whisper FastAPI service'
        ),
        
        DeclareLaunchArgument(
            'include_tutorial_system',
            default_value='false',
            description='Include complete tutorial_pkg system (set to true if not running separately)'
        ),
        
        # Configuración del sistema
        OpaqueFunction(function=launch_setup)
    ])

