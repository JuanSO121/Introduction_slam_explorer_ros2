# launch/explore_fixed.launch.py
# Launch file corregido que usa el entorno existente de Gazebo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Directorios de paquetes
    tutorial_dir = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    explore_lite_dir = FindPackageShare('explore_lite')

    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'explore_fixed.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_optimized.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    # Reescribir parámetros
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # =================================================================
    # NOTA: NO lanzamos Gazebo ni robot aquí
    # Asumimos que salon_gazebo.launch.py ya está ejecutándose
    # =================================================================

    # =================================================================
    # SLAM TOOLBOX CON CONFIGURACIÓN OPTIMIZADA
    # =================================================================
    slam_launch = TimerAction(
        period=2.0,  # Reducimos el tiempo ya que no esperamos a Gazebo
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file
                }.items()
            )
        ]
    )

    # =================================================================
    # NAVIGATION2 CON PARÁMETROS CORREGIDOS
    # =================================================================
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': configured_params
                }.items()
            )
        ]
    )

    # =================================================================
    # RVIZ OPTIMIZADO PARA EXPLORACIÓN
    # =================================================================
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'explore.rviz'])
    
    rviz_cmd = TimerAction(
        period=3.0,
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

    # =================================================================
    # MONITOR DE EXPLORACIÓN MEJORADO
    # =================================================================
    exploration_monitor = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor.py',
                name='exploration_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('/goal_pose', '/explore/goal'),
                ]
            )
        ]
    )

    # =================================================================
    # LIMPIADOR DE COSTMAPS AUTOMÁTICO
    # =================================================================
    costmap_cleaner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='costmap_cleaner.py',
                name='costmap_cleaner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # =================================================================
    # EXPLORE LITE CON CONFIGURACIÓN CORREGIDA
    # =================================================================
    explore_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore',
                output='screen',
                parameters=[configured_params],
                remappings=[
                    ('/explore/costmap', '/global_costmap/costmap'),
                    ('/explore/costmap_updates', '/global_costmap/costmap_updates'),
                ]
            )
        ]
    )

    # =================================================================
    # NODO DE DIAGNÓSTICO DE OBSTÁCULOS (CORREGIDO)
    # =================================================================
    obstacle_diagnostics = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='obstacle_diagnostics.py',
                name='obstacle_diagnostics',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # =================================================================
    # VISUALIZADOR DE ESTADO MEJORADO (SIN SKLEARN)
    # =================================================================
    exploration_visualizer = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_visualizer.py',  # Versión sin sklearn
                name='exploration_visualizer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        
        # Lanzar en secuencia optimizada (SIN Gazebo/Robot)
        slam_launch,            # t=2: SLAM
        rviz_cmd,              # t=3: RViz
        nav2_launch,           # t=5: Navigation
        exploration_monitor,    # t=7: Monitor de exploración
        costmap_cleaner,       # t=8: Limpiador automático
        explore_node,          # t=9: Exploración
        obstacle_diagnostics,   # t=10: Diagnósticos
        exploration_visualizer, # t=11: Visualizador
    ])