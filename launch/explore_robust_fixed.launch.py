#!/usr/bin/env python3

# launch/explore_robust_fixed.launch.py
# Configuración robusta que evita problemas de odometría y carga correcta

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
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def launch_setup(context, *args, **kwargs):
    """Función de configuración dinámica para verificar archivos antes del lanzamiento"""
    
    # Directorios de paquetes
    tutorial_dir = get_package_share_directory('tutorial_pkg')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world_file = LaunchConfiguration('world_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Lista de acciones a retornar
    actions = []
    
    # =================================================================
    # GAZEBO CON MUNDO PERSONALIZADO (MÉTODO ROBUSTO)
    # =================================================================
    world_path = os.path.join(tutorial_dir, 'worlds', 'salon_world.world')
    
    # Comando directo para lanzar Gazebo (más confiable)
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
    # ROBOT STATE PUBLISHER (DIRECTO, SIN DEPENDENCIAS EXTERNAS)
    # =================================================================
    
    # Encontrar el archivo URDF del TurtleBot3
    urdf_file = None
    urdf_paths_to_try = []
    
    # Intentar diferentes paquetes y rutas
    try:
        turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
        urdf_paths_to_try.extend([
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf'),
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
        ])
    except:
        pass
    
    try:
        turtlebot3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
        urdf_paths_to_try.extend([
            os.path.join(turtlebot3_cartographer_dir, 'urdf', 'turtlebot3_waffle.urdf'),
            os.path.join(turtlebot3_cartographer_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
        ])
    except:
        pass
    
    # Buscar archivo URDF válido
    robot_description = None
    for urdf_path in urdf_paths_to_try:
        if os.path.exists(urdf_path):
            try:
                with open(urdf_path, 'r') as infp:
                    robot_description_raw = infp.read()
                    # Limpiar variables no resueltas
                    robot_description = robot_description_raw.replace('${namespace}', '')
                urdf_file = urdf_path
                break
            except Exception as e:
                print(f"Error reading URDF {urdf_path}: {e}")
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
    
    # =================================================================
    # SPAWN ROBOT (MÉTODO DIRECTO SIN DEPENDENCIAS EXTERNAS)
    # =================================================================
    
    # Buscar el archivo SDF del robot
    turtlebot3_model_path = None
    possible_paths = [
        os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
        os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
        '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            turtlebot3_model_path = path
            break
    
    if turtlebot3_model_path:
        # Leer el contenido del archivo SDF
        try:
            with open(turtlebot3_model_path, 'r') as sdf_file:
                robot_sdf_content = sdf_file.read()
            
            spawn_robot_cmd = TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name='spawn_turtlebot3',
                        arguments=[
                            '-entity', 'turtlebot3',
                            '-file', turtlebot3_model_path,
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
        except Exception as e:
            print(f"Error reading SDF file: {e}")
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
                        '-z', context.perform_substitution(z_pose)
                    ],
                    output='screen'
                )
            ]
        )
        actions.append(spawn_robot_cmd)
    
    # =================================================================
    # SLAM TOOLBOX (VERIFICACIÓN DE EXISTENCIA)
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
    except Exception as e:
        print(f"Warning: SLAM Toolbox not available: {e}")
    
    # =================================================================
    # NAVIGATION2 (VERIFICACIÓN DE EXISTENCIA)
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
        else:
            print("Warning: Nav2 navigation launch file not found")
    except Exception as e:
        print(f"Warning: Nav2 not available: {e}")
    
    # =================================================================
    # EXPLORE LITE (VERIFICACIÓN DE EXISTENCIA)
    # =================================================================
    try:
        explore_lite_dir = get_package_share_directory('explore_lite')
        explore_launch_file = os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
        
        if os.path.exists(explore_launch_file):
            explore_launch = TimerAction(
                period=13.0,
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
        else:
            # Exploración directa con nodo
            explore_direct = TimerAction(
                period=13.0,
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
    except Exception as e:
        print(f"Warning: Explore Lite not available: {e}")
    
    # =================================================================
    # RVIZ (OPCIONAL CON VERIFICACIÓN)
    # =================================================================
    rviz_config_file = os.path.join(tutorial_dir, 'rviz', 'explore.rviz')
    if os.path.exists(rviz_config_file):
        rviz_cmd = TimerAction(
            period=8.0,
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
    
    # =================================================================
    # NODOS PERSONALIZADOS DE MONITOREO (VERIFICACIÓN DE EXISTENCIA)
    # =================================================================
    
    # Monitor de exploración
    monitor_script = os.path.join(tutorial_dir, 'scripts', 'exploration_monitor.py')
    if os.path.exists(monitor_script):
        exploration_monitor = TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='exploration_monitor.py',
                    name='exploration_monitor',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        actions.append(exploration_monitor)
    
    # Visualizador de exploración
    visualizer_script = os.path.join(tutorial_dir, 'scripts', 'exploration_visualizer.py')
    if os.path.exists(visualizer_script):
        exploration_visualizer = TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='exploration_visualizer.py',
                    name='exploration_visualizer',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        actions.append(exploration_visualizer)
    
    # Reiniciador de exploración
    restarter_script = os.path.join(tutorial_dir, 'scripts', 'exploration_restarter.py')
    if os.path.exists(restarter_script):
        exploration_restarter = TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='tutorial_pkg',
                    executable='exploration_restarter.py',
                    name='exploration_restarter',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        actions.append(exploration_restarter)
    
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

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(tutorial_dir, 'config', 'slam_simple.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(tutorial_dir, 'worlds', 'salon_world.world'),
        description='Full path to the Gazebo world file'
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

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        
        # Configuración dinámica
        OpaqueFunction(function=launch_setup)
    ])