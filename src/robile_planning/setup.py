import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'robile_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='md',
    maintainer_email='you@example.com',
    description='A* global planner + potential field local planner for Robile',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = robile_planning.global_planner:main',
            'potential_field_planner = robile_planning.potential_field_planner:main',
        ],
    },
)
