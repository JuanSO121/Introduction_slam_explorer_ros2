from setuptools import setup

package_name = 'tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjso',
    maintainer_email='sanchezjuanjose0508@gmail.com',
    description='Tutorial package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_monitor = tutorial_pkg.exploration_monitor:main',
        ],
    },
)

