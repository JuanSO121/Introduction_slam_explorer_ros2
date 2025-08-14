from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    # Directorios de paquetes
    tutorial_dir = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    explore_lite_dir = FindPackageShare('explore_lite')
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo')

    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    world_file = LaunchConfiguration('world_file')

    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'salon_explore.yaml']),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'slam_salon.yaml']),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'worlds', 'salon_world.world']),
        description='Full path to world file to load'
    )

    # Variables de entorno para Gazebo
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        PathJoinSubstitution([tutorial_dir, 'models'])
    )
    
    set_turtlebot3_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        'waffle'
    )

    # Parámetros reescritos
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 1. Lanzar Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gazebo', 
            '--verbose', 
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # 2. Robot State Publisher - DELAY 3s
    robot_state_publisher_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py'])
                ]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 3. Spawn del robot - DELAY 6s
    spawn_turtlebot_cmd = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-file', PathJoinSubstitution([turtlebot3_gazebo_dir, 'models', 
                                                 'turtlebot3_waffle', 'model.sdf']),
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.01',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # 4. SLAM - DELAY 9s
    slam_launch = TimerAction(
        period=9.0,
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

    # 5. RViz - DELAY 12s
    rviz_config_file = PathJoinSubstitution([tutorial_dir, 'rviz', 'salon_explore.rviz'])
    
    rviz_cmd = TimerAction(
        period=12.0,
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

    # 6. Navigation2 - DELAY 15s
    nav2_launch = TimerAction(
        period=15.0,
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

    # 7. Rotación inicial para mapeo - DELAY 18s
    initial_rotation = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='initial_mapper.py',  # ✅ Corregido: agregado .py
                name='initial_mapper',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # 8. Explore Lite - DELAY 25s (después del mapeo inicial)
    explore_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([explore_lite_dir, 'launch', 'explore.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 9. Monitor de exploración - DELAY 30s
    exploration_monitor = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_monitor.py',  # ✅ Corregido: agregado .py
                name='exploration_monitor',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Variables de entorno
        set_gazebo_model_path,
        set_turtlebot3_model,
        
        # Argumentos
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_slam_params_file_cmd,
        declare_world_file_cmd,
        
        # Secuencia de lanzamiento optimizada
        gazebo_launch,                    # 0s
        robot_state_publisher_cmd,        # 3s
        spawn_turtlebot_cmd,             # 6s
        slam_launch,                     # 9s
        rviz_cmd,                        # 12s
        nav2_launch,                     # 15s
        initial_rotation,                # 18s - Mapeo inicial
        explore_launch,                  # 25s - Exploración automática
        exploration_monitor,             # 30s - Monitor
    ])