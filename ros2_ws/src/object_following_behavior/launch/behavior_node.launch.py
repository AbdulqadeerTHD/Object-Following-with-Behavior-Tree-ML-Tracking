"""
Launch file for Behavior Tree Node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Declare arguments
    search_timeout_arg = DeclareLaunchArgument(
        'search_timeout',
        default_value='5.0',
        description='Time in seconds before switching to SEARCH mode when object is lost'
    )
    
    detection_timeout_arg = DeclareLaunchArgument(
        'detection_timeout',
        default_value='2.0',
        description='Time in seconds to confirm object detection before switching to FOLLOW'
    )
    
    # Create node
    behavior_node = Node(
        package='object_following_behavior',
        executable='behavior_node',
        name='behavior_node',
        output='screen',
        parameters=[{
            'search_timeout': LaunchConfiguration('search_timeout'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
        }]
    )
    
    return LaunchDescription([
        search_timeout_arg,
        detection_timeout_arg,
        behavior_node,
    ])

