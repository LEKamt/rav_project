from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikolay',
    maintainer_email='cruzattnikolay@gmail.com',
    description='Puesta en marcha del dron F450',
    license='MIT',
    entry_points={
        'console_scripts': [
            'estacion_tierra = robot_bringup.estacion_tierra:main',
            'monitor = robot_bringup.monitor:main',
            'puente_firmware = robot_bringup.puente_firmware:main',
        ],
    },
)
