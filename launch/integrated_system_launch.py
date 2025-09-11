#!/usr/bin/env python3
"""
Launch file integrado con coordinador central - VERSIÓN CORREGIDA
Implementa el patrón coordinador para evitar conflictos entre servicios
Ubicación: ~/ros2_ws/src/tutorial_pkg/launch/integrated_system_launch_fixed.py
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
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import subprocess


def get_robot_description():
    """Obtener la descripción del robot de forma robusta"""
    
    # Método 1: Intentar con xacro (método correcto)
    try:
        turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
        xacro_file = os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf.xacro')
        
        if os.path.exists(xacro_file):
            print(f"✅ Encontrado XACRO: {xacro_file}")
            robot_description_command = Command([
                'xacro ', xacro_file
            ])
            return robot_description_command
    except Exception as e:
        print(f"⚠️ Error con xacro: {e}")
    
    # Método 2: Buscar archivo URDF directo
    try:
        turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
        urdf_paths = [
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf'),
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf'),
            os.path.join(turtlebot3_description_dir, 'robots', 'turtlebot3_waffle.urdf')
        ]
        
        for urdf_path in urdf_paths:
            if os.path.exists(urdf_path):
                print(f"✅ Encontrado URDF: {urdf_path}")
                with open(urdf_path, 'r') as infp:
                    robot_description = infp.read()
                return robot_description
    except Exception as e:
        print(f"⚠️ Error con URDF: {e}")
    
    # Método 3: URDF mínimo como fallback
    print("⚠️ Usando URDF básico como fallback")
    return '''<?xml version="1.0"?>
<robot name="turtlebot3_waffle" xmlns:xacro="http://ros.org/wiki/xacro">
  <link name="base_footprint">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.010" rpy="0 0 0"/>
  </joint>
  
  <link name="base_link">
    <visual>
      <origin xyz="-0.064 0 0.0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/waffle_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black"/>
    </visual>
    <collision>
      <origin xyz="-0.064 0 0.047" rpy="0 0 0"/>
      <geometry>
        <box size="0.266 0.266 0.094"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0.0 0.144 0.023" rpy="-1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <link name="wheel_left_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0.0 -0.144 0.023" rpy="-1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <link name="wheel_right_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.57 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.018" radius="0.033"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="-0.064 0 0.122" rpy="0 0 0"/>
  </joint>
  
  <link name="base_scan">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/sensors/lds.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0.015 0 -0.0065" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.0315" radius="0.055"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.114"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <material name="light_black">
    <color rgba="0.4 0.4 0.4 1.0"/>
  </material>
  
  <material name="dark">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>
  
  <gazebo reference="wheel_left_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>
  
  <gazebo reference="wheel_right_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>
  
  <gazebo>
    <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.287</wheel_separation>
      <wheel_diameter>0.066</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>
  
  <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_ray_sensor.so">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>'''


def launch_setup(context, *args, **kwargs):
    """Configuración del sistema integrado con coordinador central - CORREGIDA"""

    tutorial_dir = get_package_share_directory('tutorial_pkg')
    config_file = os.path.join(tutorial_dir, 'config', 'integrated_system_config.yaml')

    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    robot_model = LaunchConfiguration('robot_model')
    enable_voice_control = LaunchConfiguration('enable_voice_control')
    enable_flutter_bridge = LaunchConfiguration('enable_flutter_bridge')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    actions = []

    # =================================================================
    # 1. GAZEBO Y ROBOT (CONFIGURACIÓN BASE) - MÉTODO ROBUSTO
    # =================================================================

    # Gazebo (servidor + cliente GUI)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    actions.extend([gzserver, gzclient])

    # Robot State Publisher (MÉTODO CORREGIDO)
    robot_description_content = get_robot_description()
    
    robot_state_publisher = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description_content
                }]
            )
        ]
    )
    actions.append(robot_state_publisher)
    print("✅ Robot State Publisher configurado correctamente")

    # Joint State Publisher (AÑADIDO - IMPORTANTE)
    joint_state_publisher = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    actions.append(joint_state_publisher)

    # Spawn Robot (MÉTODO MEJORADO)
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', 'turtlebot3',
                    '-topic', '/robot_description',
                    '-x', x_pose, '-y', y_pose, '-z', z_pose,
                    '-timeout', '20.0'  # Añadir timeout
                ],
                output='screen'
            )
        ]
    )
    actions.append(spawn_robot)
    
    # =================================================================
    # 2. SLAM Y NAVEGACIÓN
    # =================================================================
    try:
        slam_launch = TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[config_file, {'use_sim_time': use_sim_time}]
                )
            ]
        )
        actions.append(slam_launch)
    except Exception:
        print("⚠️ SLAM Toolbox not available")

    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

        if os.path.exists(nav_launch_file):
            nav2_launch = TimerAction(
                period=10.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch_file),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'params_file': config_file
                        }.items()
                    )
                ]
            )
            actions.append(nav2_launch)
    except Exception:
        print("⚠️ Nav2 not available")
    
    # =================================================================
    # 3. COORDINADOR CENTRAL - NÚCLEO DEL SISTEMA
    # =================================================================
    
    robot_coordinator = TimerAction(
        period=12.0,  # Dar más tiempo para que el robot se spawne
        actions=[
            Node(
                package='tutorial_pkg',
                executable='robot_control_coordinator',
                name='robot_control_coordinator',
                output='screen',
                parameters=[config_file],
                remappings=[
                    # El coordinador es quien publica a /cmd_vel
                    ('/cmd_vel', '/cmd_vel'),
                    # Recibe solicitudes de diferentes fuentes
                    ('/voice_commands', '/voice_commands'),
                    ('/manual_control', '/manual_control'),
                    ('/exploration_request', '/exploration_request'),
                    ('/emergency_command', '/emergency_command'),
                    # Publica estado y feedback
                    ('/coordinator_status', '/coordinator_status'),
                    ('/control_feedback', '/control_feedback')
                ]
            )
        ]
    )
    actions.append(robot_coordinator)
    
    # =================================================================
    # 4. MONITOR DE EXPLORACIÓN MEJORADO
    # =================================================================
    
    enhanced_monitor = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='enhanced_exploration_monitor',
                name='enhanced_exploration_monitor',
                output='screen',
                parameters=[config_file],
                remappings=[
                    # Lee sensores
                    ('/map', '/map'),
                    ('/odom', '/odom'),
                    ('/scan', '/scan'),
                    # Se comunica con coordinador
                    ('/exploration_request', '/exploration_request'),
                    ('/coordinator_status', '/coordinator_status'),
                    ('/monitor_status', '/monitor_status')
                ]
            )
        ]
    )
    actions.append(enhanced_monitor)
    
    # =================================================================
    # 5. CONTROL POR VOZ INTEGRADO (CONDICIONAL)
    # =================================================================
    
    voice_control_group = GroupAction(
        condition=IfCondition(enable_voice_control),
        actions=[
            TimerAction(
                period=16.0,
                actions=[
                    Node(
                        package='tutorial_pkg',
                        executable='integrated_ai_voice_commander',
                        name='integrated_ai_voice_commander',
                        output='screen',
                        parameters=[config_file],
                        remappings=[
                            # Recibe input de voz crudo
                            ('/raw_voice_input', '/raw_voice_input'),
                            # Envía comandos procesados al coordinador
                            ('/voice_commands', '/voice_commands'),
                            # Monitorea estado del coordinador
                            ('/coordinator_status', '/coordinator_status'),
                            # Publica feedback
                            ('/voice_feedback', '/voice_feedback'),
                            ('/control_feedback', '/control_feedback')
                        ]
                    )
                ]
            )
        ]
    )
    actions.append(voice_control_group)
    
    # =================================================================
    # 6. FLUTTER BRIDGE (CONDICIONAL)
    # =================================================================
    
    flutter_bridge_group = GroupAction(
        condition=IfCondition(enable_flutter_bridge),
        actions=[
            TimerAction(
                period=18.0,
                actions=[
                    Node(
                        package='tutorial_pkg',
                        executable='flutter_bridge_node',
                        name='flutter_bridge_node',
                        output='screen',
                        parameters=[config_file],
                        remappings=[
                            # Interface con coordinador
                            ('/raw_voice_input', '/raw_voice_input'),
                            ('/manual_control', '/manual_control'),
                            ('/coordinator_status', '/coordinator_status'),
                            ('/control_feedback', '/control_feedback'),
                            # Estado del sistema
                            ('/voice_feedback', '/voice_feedback'),
                            ('/monitor_status', '/monitor_status')
                        ]
                    )
                ]
            )
        ]
    )
    actions.append(flutter_bridge_group)
    
    # =================================================================
    # 7. EXPLORE LITE - SISTEMA DE EXPLORACIÓN
    # =================================================================
    
    try:
        explore_lite = TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='explore_lite',
                    executable='explore',
                    name='explore_node',
                    output='screen',
                    parameters=[config_file],
                    remappings=[
                        ('/cmd_vel', '/exploration_cmd_vel')  # NO conflicto con coordinador
                    ]
                )
            ]
        )
        actions.append(explore_lite)
    except:
        print("Warning: Explore Lite not available")
    
    # =================================================================
    # 8. BRIDGE ENTRE EXPLORE LITE Y COORDINADOR
    # =================================================================
    
    exploration_bridge = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_bridge',
                name='exploration_bridge',
                output='screen',
                parameters=[config_file],
                remappings=[
                    # Intercepta comandos de explore_lite
                    ('/exploration_cmd_vel', '/exploration_cmd_vel'),
                    # Los reenvía como solicitudes al coordinador
                    ('/exploration_request', '/exploration_request'),
                    # Monitorea estado
                    ('/coordinator_status', '/coordinator_status')
                ]
            )
        ]
    )
    actions.append(exploration_bridge)
    
    # =================================================================
    # 9. VISUALIZACIÓN Y MONITOREO
    # =================================================================
    
    # RViz
    rviz_config = os.path.join(tutorial_dir, 'rviz', 'integrated_exploration.rviz')
    if os.path.exists(rviz_config):
        rviz_launch = TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        )
        actions.append(rviz_launch)
    
    # Visualizador de exploración
    exploration_viz = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='exploration_visualizer',
                name='exploration_visualizer',
                output='screen',
                parameters=[config_file]
            )
        ]
    )
    actions.append(exploration_viz)
    
    print(f"Sistema integrado configurado con {len(actions)} componentes")
    print("ARQUITECTURA CORREGIDA:")
    print("  Robot State Publisher -> Publica descripción del robot")
    print("  Joint State Publisher -> Publica estados de joints")
    print("  Spawn Entity -> Spawna robot en Gazebo")
    print("  Coordinador Central -> Controla todo el movimiento")
    print("  Monitor Exploración -> Solicita acciones al coordinador")
    print("  Voice Commander -> Envía comandos al coordinador")
    print("  Flutter Bridge -> Interface con coordinador")
    print("  Explore Lite -> Bridgeado al coordinador")

    return actions


def generate_launch_description():
    """Generar descripción de lanzamiento para sistema integrado - VERSIÓN CORREGIDA"""

    tutorial_dir = get_package_share_directory('tutorial_pkg')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_file',
            default_value=os.path.join(tutorial_dir, 'worlds', 'salon_world.world')),
        DeclareLaunchArgument('robot_model', default_value='waffle'),
        DeclareLaunchArgument('enable_voice_control', default_value='true'),
        DeclareLaunchArgument('enable_flutter_bridge', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        OpaqueFunction(function=launch_setup)
    ])