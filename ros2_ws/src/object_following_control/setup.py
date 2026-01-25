from setuptools import setup
import os
from glob import glob

package_name = 'object_following_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='team@example.com',
    description='PID-based control node for object following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = object_following_control.control_node:main',
            'controller_node = object_following_control.controller_node:main',
            'keep_alive = object_following_control.keep_alive_node:main',
            'teleop_recorder = object_following_control.teleop_recorder_node:main',
        ],
    },
)

