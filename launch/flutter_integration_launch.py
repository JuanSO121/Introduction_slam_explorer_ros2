#!/usr/bin/env python3
"""
Launch file para integrar Flutter con tutorial_pkg SIN MODIFICAR el sistema existente
Ubicación: ~/ros2_ws/src/tutorial_pkg/launch/flutter_integration_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Configurar la integración Flutter manteniendo tutorial_pkg intacto"""
    
    # Directorios
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    
    # Configuraciones
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_flutter_bridge = LaunchConfiguration('enable_flutter_bridge')
    flask_port = LaunchConfiguration('flask_port')
    enable_voice = LaunchConfiguration('enable_voice')
    
    # Convertir flask_port a entero para evitar error de tipo
    flask_port_value = int(context.perform_substitution(flask_port))
    enable_voice_value = context.perform_substitution(enable_voice).lower() == 'true'
    
    actions = []
    
    # =================================================================
    # 1. LANZAR EL SISTEMA TUTORIAL_PKG COMPLETO (SIN MODIFICACIONES)
    # =================================================================
    
    # Incluir el launch principal de tutorial_pkg
    tutorial_main_launch = os.path.join(tutorial_dir, 'launch', 'integrated_exploration_launch.py')
    
    if os.path.exists(tutorial_main_launch):
        tutorial_launch_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tutorial_main_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_voice_control': 'true',  # Mantener el sistema de voz existente
                'enable_flask_server': 'false',  # No usar el Flask del tutorial original
                'x_pose': '0.0',
                'y_pose': '0.0',
                'z_pose': '0.01'
            }.items()
        )
        actions.append(tutorial_launch_action)
        print("✅ Sistema tutorial_pkg incluido")
    else:
        print("⚠️ Launch principal de tutorial_pkg no encontrado")
        print(f"   Buscando en: {tutorial_main_launch}")
        
        # Buscar otros posibles launch files
        launch_dir = os.path.join(tutorial_dir, 'launch')
        if os.path.exists(launch_dir):
            print("   Archivos launch disponibles:")
            for file in os.listdir(launch_dir):
                if file.endswith('.py'):
                    print(f"   - {file}")
    
    # =================================================================
    # 2. AGREGAR EL FLUTTER BRIDGE NODE (NUEVO COMPONENTE)
    # =================================================================
    
    # Configuración del bridge
    flutter_config_file = os.path.join(tutorial_dir, 'config', 'flutter_bridge_config.yaml')
    
    flutter_bridge_node = TimerAction(
        period=5.0,  # Esperar a que tutorial_pkg esté listo
        actions=[
            Node(
                package='tutorial_pkg',
                executable='flutter_bridge_node',
                name='flutter_bridge_node',
                output='screen',
                parameters=[
                    flutter_config_file if os.path.exists(flutter_config_file) else {},
                    {
                        'use_sim_time': use_sim_time,
                        'flask_port': flask_port_value,  # Usar valor entero directamente
                        'enable_voice_processing': enable_voice_value,  # Usar valor booleano
                        'enable_websocket': True
                    }
                ],
                condition=IfCondition(enable_flutter_bridge)
            )
        ]
    )
    actions.append(flutter_bridge_node)
    
    # =================================================================
    # 3. ROSBRIDGE SERVER PARA WEBSOCKET (SI NO ESTÁ CORRIENDO)
    # =================================================================
    
    rosbridge_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                output='screen',
                parameters=[{
                    'port': 9090,
                    'address': '0.0.0.0'
                }],
                condition=IfCondition(enable_flutter_bridge)
            )
        ]
    )
    actions.append(rosbridge_node)
    
    # =================================================================
    # 4. MONITOR DE CONEXIÓN FLUTTER (OPCIONAL)
    # =================================================================
    
    # Comentado por ahora para evitar conflictos
    # flutter_monitor_node = TimerAction(
    #     period=10.0,
    #     actions=[
    #         Node(
    #             package='tutorial_pkg',
    #             executable='flutter_bridge_node',
    #             name='flutter_connection_monitor',
    #             output='screen',
    #             parameters=[{
    #                 'monitor_only': True,
    #                 'flask_port': flask_port_value
    #             }],
    #             condition=IfCondition(enable_flutter_bridge)
    #         )
    #     ]
    # )
    
    print("✅ Flutter Bridge configurado")
    print(f"🌐 Servidor Flask estará disponible en puerto {flask_port_value}")
    print("🌉 Rosbridge WebSocket en puerto 9090")
    
    return actions


def generate_launch_description():
    """Generar descripción de lanzamiento para integración Flutter"""
    
    return LaunchDescription([
        # Argumentos de configuración
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'enable_flutter_bridge',
            default_value='true',
            description='Enable Flutter bridge node'
        ),
        
        DeclareLaunchArgument(
            'flask_port',
            default_value='8000',
            description='Port for Flask server'
        ),
        
        DeclareLaunchArgument(
            'enable_voice',
            default_value='true',
            description='Enable voice processing in bridge'
        ),
        
        DeclareLaunchArgument(
            'world_file',
            default_value='salon_world.world',
            description='Gazebo world file'
        ),
        
        DeclareLaunchArgument(
            'robot_model',
            default_value='waffle',
            description='TurtleBot3 model'
        ),
        
        # Configuración del sistema
        OpaqueFunction(function=launch_setup)
    ])