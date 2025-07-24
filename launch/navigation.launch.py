import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    # Directorios
    pkg_share = FindPackageShare('tutorial_pkg')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    # Argumentos de lanzamiento
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_share, 'maps', 'mi_primer_mapa.yaml']),
        description='Full path to map yaml file to load')

    # Reescribir parámetros
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Lanzar Gazebo + TurtleBot3
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Lanzar Nav2
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])),
        launch_arguments={'namespace': namespace,
                         'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'params_file': configured_params,
                         'map': map_yaml_file}.items())

    # RViz
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'nav2_config.rviz'])
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Crear y devolver descripción del lanzamiento
    ld = LaunchDescription()

    # Declarar argumentos
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Añadir acciones
    ld.add_action(gazebo_launch)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
