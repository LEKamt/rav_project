from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test_scenarios', glob('test_scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikolay',
    maintainer_email='cruzattnikolay@gmail.com',
    description='Pruebas del control del dron F450',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'corredor = robot_tests.corredor:main',
        ],
    },
)
