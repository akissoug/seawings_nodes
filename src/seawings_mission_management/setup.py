from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'seawings_mission_management'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akissoug',
    maintainer_email='asougles@gmail.com',
    description='ROS 2 mission management subsystem for SEAWINGS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_monitor = seawings_mission_management.power_monitor:main',
            'fault_detector = seawings_mission_management.fault_detector:main',
            'mission_supervisor = seawings_mission_management.mission_supervisor:main',
        ],
    },
)
