from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tutorial_pkg'

def get_data_files():
    """Función para obtener todos los archivos de datos recursivamente"""
    data_files = []
    
    # Archivos básicos del paquete
    data_files.extend([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ])
    
    # Launch files
    launch_files = glob('launch/*.py')
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), launch_files))
    
    # Config files
    config_files = glob('config/*.yaml')
    if config_files:
        data_files.append((os.path.join('share', package_name, 'config'), config_files))
    
    # World files
    world_files = glob('worlds/*.world')
    if world_files:
        data_files.append((os.path.join('share', package_name, 'worlds'), world_files))
    
    # RViz files
    rviz_files = glob('rviz/*.rviz')
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), rviz_files))
    
    # Maps files
    maps_files = glob('maps/*')
    if maps_files:
        data_files.append((os.path.join('share', package_name, 'maps'), maps_files))
    
    # Scripts
    script_files = glob('scripts/*.py') + glob('scripts/*.sh')
    if script_files:
        data_files.append((os.path.join('share', package_name, 'scripts'), script_files))
    
    # Models (recursivamente)
    for root, dirs, files in os.walk('models'):
        if files:
            target_dir = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, file_paths))
    
    return data_files

setup(
    name=package_name,
    version='1.0.0',
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
    description='Paquete tutorial para exploración robusta con TurtleBot3 en ambiente de salón',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_monitor = tutorial_pkg.exploration_monitor:main',
            'exploration_restarter = tutorial_pkg.exploration_restarter:main',
            'exploration_visualizer = tutorial_pkg.exploration_visualizer:main',
            'initial_mapper = tutorial_pkg.initial_mapper:main',
            'costmap_cleaner = tutorial_pkg.costmap_cleaner:main',
        ],
    },
)