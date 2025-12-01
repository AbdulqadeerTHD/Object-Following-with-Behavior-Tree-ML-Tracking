"""
Launch file for the YOLOv8 Tracker Vision Node

This launch file starts the vision node that performs object detection and tracking.
It can be included in a larger launch file for the complete robot system.

Usage:
    ros2 launch object_following_vision vision_node.launch.py
    ros2 launch object_following_vision vision_node.launch.py camera_topic:=/tb3/camera/image_raw target_class_name:=person
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the vision node.
    """
    
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='ROS2 topic name for camera images (e.g., /camera/image_raw or /tb3/camera/image_raw)'
    )
    
    target_class_arg = DeclareLaunchArgument(
        'target_class_name',
        default_value='person,chair,box,dining table,bottle',
        description='Target object classes to detect and track (comma-separated, e.g., person,chair,box,dining table,bottle). Priority: person > chair > table > box > bottle'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.4',
        description='YOLOv8 detection confidence threshold (0.0 to 1.0)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLOv8 model file (yolov8n.pt, yolov8s.pt, yolov8m.pt, etc.)'
    )
    
    max_age_arg = DeclareLaunchArgument(
        'max_age',
        default_value='30',
        description='DeepSORT max_age: maximum frames to keep a lost track'
    )
    
    n_init_arg = DeclareLaunchArgument(
        'n_init',
        default_value='3',
        description='DeepSORT n_init: minimum detections before confirming a track'
    )
    
    # Create the vision node
    vision_node = Node(
        package='object_following_vision',
        executable='yolov8_tracker',
        name='yolov8_tracker_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'target_class_name': LaunchConfiguration('target_class_name'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_path': LaunchConfiguration('model_path'),
            'max_age': LaunchConfiguration('max_age'),
            'n_init': LaunchConfiguration('n_init'),
        }],
        remappings=[
            # You can remap topics here if needed
            # ('/camera/image_raw', '/tb3/camera/image_raw'),
        ]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        target_class_arg,
        confidence_threshold_arg,
        model_path_arg,
        max_age_arg,
        n_init_arg,
        vision_node,
    ])

