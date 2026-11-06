from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/param', glob('param/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikolay',
    maintainer_email='cruzattnikolay@gmail.com',
    description='Misiones de altura del dron F450',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mision_altura = robot_navigation.mision_altura:main',
        ],
    },
)
