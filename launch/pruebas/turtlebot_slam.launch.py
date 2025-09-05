from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Argumentos
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world_name', 
        default_value='turtlebot3_world', 
        description='Gazebo world name (turtlebot3_world, turtlebot3_house, empty_world)'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare("tutorial_pkg"), 'config', 'slam_turtlebot.yaml'
        ]),
        description='Full path to the ROS2 parameters file for slam_toolbox',
    )

    # Incluir launch de TurtleBot3 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'  # Cambiar por empty_world.launch.py o turtlebot3_house.launch.py si se desea
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Nodo SLAM
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz con configuración predeterminada
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('turtlebot3_gazebo'), 'rviz', 'tb3_gazebo.rviz'
        ])]
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        slam_params_file_arg,
        gazebo_launch,
        slam_node,
        rviz_node,
    ])