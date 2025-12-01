"""
Launch file for PID-based Control Node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Declare arguments
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.3',
        description='Maximum linear velocity [m/s]'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity [rad/s]'
    )
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='0.5',
        description='Target following distance [m]'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.4',
        description='LiDAR obstacle detection threshold [m]'
    )
    
    # Create node
    control_node = Node(
        package='object_following_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'target_distance': LaunchConfiguration('target_distance'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
        }]
    )
    
    return LaunchDescription([
        max_linear_vel_arg,
        max_angular_vel_arg,
        target_distance_arg,
        obstacle_threshold_arg,
        control_node,
    ])

