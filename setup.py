from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tutorial_pkg'

def get_data_files():
    data_files = []
    
    # Archivos base del paquete
    data_files.extend([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ])

    # Directorios a excluir
    EXCLUDE_DIRS = {'venv', '__pycache__', '.git', '.pytest_cache', 'node_modules', '.egg-info', 'build', 'dist'}
    EXCLUDE_PATTERNS = {'.pyc', '.pyo', '.pyd', '.DS_Store', 'Thumbs.db', '.tmp', '.backup'}

    def should_exclude(path):
        parts = set(path.split(os.sep))
        if parts.intersection(EXCLUDE_DIRS):
            return True
        
        filename = os.path.basename(path)
        for pattern in EXCLUDE_PATTERNS:
            if pattern.startswith('*') and filename.endswith(pattern[1:]):
                return True
            elif pattern.endswith('*') and filename.startswith(pattern[:-1]):
                return True
            elif pattern == filename:
                return True
        return False

    # Archivos de launch
    launch_files = [f for f in glob('launch/*.py') if not should_exclude(f)]
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

    # Archivos de configuración
    config_files = [f for f in glob('config/*.yaml') if not should_exclude(f)]
    if config_files:
        data_files.append((os.path.join('share', package_name, 'config'), config_files))

    # Mundos de Gazebo
    world_files = [f for f in glob('worlds/*.world') if not should_exclude(f)]
    if world_files:
        data_files.append((os.path.join('share', package_name, 'worlds'), world_files))

    # Configuraciones de RViz
    rviz_files = [f for f in glob('rviz/*.rviz') if not should_exclude(f)]
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), rviz_files))

    # Mapas generados
    map_extensions = ['*.pgm', '*.yaml']
    maps_files = []
    for ext in map_extensions:
        maps_files.extend([f for f in glob(f'maps/{ext}') if not should_exclude(f)])
    if maps_files:
        data_files.append((os.path.join('share', package_name, 'maps'), maps_files))

    # Scripts de utilidad
    script_files = [f for f in glob('scripts/*.py') + glob('scripts/*.sh') if not should_exclude(f)]
    if script_files:
        data_files.append((os.path.join('share', package_name, 'scripts'), script_files))

    # Modelos 3D (si existen)
    if os.path.exists('models'):
        for root, dirs, files in os.walk('models'):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            if files:
                filtered_files = [f for f in files if not should_exclude(os.path.join(root, f))]
                if filtered_files:
                    target_dir = os.path.join('share', package_name, root)
                    file_paths = [os.path.join(root, f) for f in filtered_files]
                    data_files.append((target_dir, file_paths))

    # Servicios de voz (si existen)
    if os.path.exists('voice_services'):
        for root, dirs, files in os.walk('voice_services'):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            if files:
                filtered_files = [f for f in files if not should_exclude(os.path.join(root, f)) and f.endswith('.py')]
                if filtered_files:
                    target_dir = os.path.join('share', package_name, root)
                    file_paths = [os.path.join(root, f) for f in filtered_files]
                    data_files.append((target_dir, file_paths))


    return data_files

setup(
    name=package_name,
    version='3.0.0',  # Versión mayor por arquitectura integrada
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=[
        'setuptools',
        # ROS2 core
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'std_msgs',
        'sensor_msgs',
        'nav2_msgs',
        'visualization_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        # Procesamiento de datos
        'numpy>=1.19.0',
        'scipy>=1.6.0',
        'opencv-python>=4.5.0',
        # Servidor web para Flutter
        'flask>=2.0.0',
        'flask-cors>=4.0.0',
        'flask-socketio>=5.0.0',
        'python-socketio>=5.0.0',
        # HTTP y requests
        'requests>=2.25.0',
        'werkzeug>=2.0.0',
        # Utilidades
        'pyyaml>=5.4.0',
        'python-dateutil>=2.8.0'
    ],
    extras_require={
        'voice': [
            'openai-whisper>=20230314',
            'torch>=1.9.0',
            'torchaudio>=0.9.0',
            'transformers>=4.20.0',
            'librosa>=0.9.0',
            'soundfile>=0.10.0'
        ],
        'ai': [
            'google-generativeai>=0.3.0',
            'openai>=0.27.0'
        ],
        'dev': [
            'pytest>=6.0.0',
            'pytest-cov>=3.0.0',
            'flake8>=4.0.0',
            'black>=22.0.0'
        ],
        'monitoring': [
            'psutil>=5.8.0',
            'matplotlib>=3.5.0'
        ]
    },
    zip_safe=True,
    maintainer='Juan Sanchez',
    maintainer_email='sanchezjuanjo0508@gmail.com',
    description='Sistema integrado de exploración robótica con TurtleBot3, IA y control coordinado',
    long_description="""
    Tutorial PKG v3.0 - Sistema Integrado de Exploración Robótica
    
    Características principales:
    - Coordinador central para evitar conflictos entre servicios
    - Sistema de control por voz con IA integrada
    - Monitoreo inteligente de exploración
    - Interfaz Flutter para control móvil
    - Arquitectura basada en máquina de estados
    - Integración completa con Nav2 y SLAM
    
    Componentes principales:
    - RobotControlCoordinator: Núcleo del sistema de control
    - EnhancedExplorationMonitor: Monitoreo inteligente
    - IntegratedAIVoiceCommander: Control por voz coordinado
    - FlutterBridgeNode: Interface móvil
    - ExplorationBridge: Intermediario con explore_lite
    """,
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Componentes del sistema integrado (NUEVOS)
            'robot_control_coordinator = tutorial_pkg.robot_control_coordinator:main',
            'enhanced_exploration_monitor = tutorial_pkg.enhanced_exploration_monitor:main',
            'integrated_ai_voice_commander = tutorial_pkg.integrated_ai_voice_commander:main',
            'exploration_bridge = tutorial_pkg.exploration_bridge:main',
            
            # Componentes de compatibilidad (ACTUALIZADOS)
            'flutter_bridge_node = tutorial_pkg.flutter_bridge_node:main',
            'ai_response_node = tutorial_pkg.ai_response_node:main',
            
            # Componentes heredados (MANTENIDOS por compatibilidad)
            'exploration_monitor = tutorial_pkg.exploration_monitor:main',
            'ai_voice_commander = tutorial_pkg.ai_voice_commander:main',
            'exploration_restarter = tutorial_pkg.exploration_restarter:main',
            'exploration_visualizer = tutorial_pkg.exploration_visualizer:main',
            'initial_mapper = tutorial_pkg.initial_mapper:main',
            'costmap_cleaner = tutorial_pkg.costmap_cleaner:main',
            'obstacle_diagnostics = tutorial_pkg.obstacle_diagnostics:main',
            
            'whisper_fastapi_service = tutorial_pkg.whisper_fastapi_service:main',
            'exploration_arbiter = tutorial_pkg.exploration_arbiter:main',

        ],
    },
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Robotics',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    keywords='robotics, ros2, turtlebot3, slam, navigation, ai, voice-control, flutter, exploration',
)