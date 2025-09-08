from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tutorial_pkg'

def get_data_files():
    """Función para obtener archivos de datos, excluyendo problemáticos"""
    data_files = []
    
    # Archivos básicos del paquete
    data_files.extend([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ])
    
    # Directorios y archivos a excluir
    EXCLUDE_DIRS = {'venv', '__pycache__', '.git', '.pytest_cache', 'node_modules', '.egg-info'}
    EXCLUDE_PATTERNS = {'*.pyc', '*.pyo', '*.pyd', '.DS_Store', 'Thumbs.db', '*.tmp', '*.backup*'}
    
    def should_exclude(path):
        """Verificar si un path debe ser excluido"""
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
    
    # Launch files
    launch_files = [f for f in glob('launch/*.py') if not should_exclude(f)]
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), launch_files))
    
    # Config files
    config_files = [f for f in glob('config/*.yaml') if not should_exclude(f)]
    if config_files:
        data_files.append((os.path.join('share', package_name, 'config'), config_files))
    
    # World files
    world_files = [f for f in glob('worlds/*.world') if not should_exclude(f)]
    if world_files:
        data_files.append((os.path.join('share', package_name, 'worlds'), world_files))
    
    # RViz files
    rviz_files = [f for f in glob('rviz/*.rviz') if not should_exclude(f)]
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), rviz_files))
    
    # Maps files (solo archivos específicos)
    map_extensions = ['*.pgm', '*.yaml']
    maps_files = []
    for ext in map_extensions:
        maps_files.extend([f for f in glob(f'maps/{ext}') if not should_exclude(f)])
    if maps_files:
        data_files.append((os.path.join('share', package_name, 'maps'), maps_files))
    
    # Scripts (solo archivos específicos, no directorios)
    script_files = [f for f in glob('scripts/*.py') + glob('scripts/*.sh') if not should_exclude(f)]
    if script_files:
        data_files.append((os.path.join('share', package_name, 'scripts'), script_files))
    
    # Models (con filtrado mejorado)
    if os.path.exists('models'):
        for root, dirs, files in os.walk('models'):
            # Excluir directorios problemáticos
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            if files:
                # Filtrar archivos
                filtered_files = [f for f in files if not should_exclude(os.path.join(root, f))]
                if filtered_files:
                    target_dir = os.path.join('share', package_name, root)
                    file_paths = [os.path.join(root, f) for f in filtered_files]
                    data_files.append((target_dir, file_paths))
    
    return data_files

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'std_msgs',
        'sensor_msgs',
        'nav2_msgs',
        'visualization_msgs'
    ],
    zip_safe=True,
    maintainer='Juan Sanchez',
    maintainer_email='sanchezjuanjo0508@gmail.com',
    description='Paquete tutorial para exploración con TurtleBot3 e integración IA',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodos originales
            'exploration_monitor = tutorial_pkg.exploration_monitor:main',
            'exploration_restarter = tutorial_pkg.exploration_restarter:main',
            'exploration_visualizer = tutorial_pkg.exploration_visualizer:main',
            'initial_mapper = tutorial_pkg.initial_mapper:main',
            'costmap_cleaner = tutorial_pkg.costmap_cleaner:main',
            'obstacle_diagnostics = tutorial_pkg.obstacle_diagnostics:main',
            'enhanced_exploration_monitor = tutorial_pkg.enhanced_exploration_monitor:main',
            # Nodos con integración IA
            'ai_response_node = tutorial_pkg.ai_response_node:main',
            'ai_voice_commander = tutorial_pkg.ai_voice_commander:main',
            # 'voice_command_handler = tutorial_pkg.voice_command_handler:main',
        ],
    },
)