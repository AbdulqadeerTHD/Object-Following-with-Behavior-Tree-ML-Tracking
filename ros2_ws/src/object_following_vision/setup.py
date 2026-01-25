from setuptools import setup
import os
from glob import glob

package_name = 'object_following_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,  # Must be False for message generation
    maintainer='Team Member 1',
    maintainer_email='team@example.com',
    description='ROS2 vision package for object detection and tracking using YOLOv8 and DeepSORT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_tracker = object_following_vision.yolov8_tracker_node:main',
            'data_collector = object_following_vision.data_collector_node:main',
            'vision_yolo = object_following_vision.vision_yolo_node:main',
        ],
    },
)

