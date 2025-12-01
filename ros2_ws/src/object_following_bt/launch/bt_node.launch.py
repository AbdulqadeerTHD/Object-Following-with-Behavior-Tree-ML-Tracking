"""
Launch file for Behavior Tree Node
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Create node
    bt_node = Node(
        package='object_following_bt',
        executable='bt_node',
        name='bt_node',
        output='screen',
    )
    
    return LaunchDescription([
        bt_node,
    ])
