"""
Launch file for final_world.wbt - Uses the final world environment with current robot
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # Get package directories
    integration_package_dir = get_package_share_directory('object_following_integration')
    turtlebot_package_dir = get_package_share_directory('webots_ros2_turtlebot')
    
    # Declare arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic name'
    )
    
    target_class_arg = DeclareLaunchArgument(
        'target_class_name',
        default_value='person',
        description='Target object class to detect (person, chair, bottle, all, etc.) - Default: person'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.2',
        description='YOLOv8 confidence threshold (0.0 to 1.0)'
    )
    
    # Get world file path - FINAL WORLD FILE
    world_file = os.path.join(integration_package_dir, 'worlds', 'final_world.wbt')
    
    # Launch Webots with final world
    webots = WebotsLauncher(
        world=world_file,
        mode='realtime',
        ros2_supervisor=True
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }]
    )
    
    # Static transform publisher
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    
    # Webots controller - use URDF with camera configuration
    custom_urdf_installed = os.path.join(integration_package_dir, 'resource', 'turtlebot_webots_with_camera.urdf')
    try:
        source_base = os.path.dirname(os.path.dirname(os.path.dirname(integration_package_dir)))
        custom_urdf_source = os.path.join(source_base, 'src', 'object_following_integration', 'resource', 'turtlebot_webots_with_camera.urdf')
    except:
        custom_urdf_source = None
    default_urdf_path = os.path.join(turtlebot_package_dir, 'resource', 'turtlebot_webots.urdf')
    
    # Choose URDF path (prefer custom with camera)
    if os.path.exists(custom_urdf_installed):
        robot_description_path = custom_urdf_installed
    elif custom_urdf_source and os.path.exists(custom_urdf_source):
        robot_description_path = custom_urdf_source
    else:
        robot_description_path = default_urdf_path
    
    # ROS2 control parameters
    ros2_control_params = os.path.join(turtlebot_package_dir, 'resource', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ.get('ROS_DISTRO') in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    
    # WebotsController
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )
    
    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    
    # Wait for controller connection
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=[diffdrive_controller_spawner, joint_state_broadcaster_spawner]
    )
    
    # Vision node
    vision_node = Node(
        package='object_following_vision',
        executable='yolov8_tracker',
        name='vision_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'target_class_name': LaunchConfiguration('target_class_name'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }]
    )
    
    # Behavior Tree node
    bt_node = Node(
        package='object_following_bt',
        executable='bt_node',
        name='bt_node',
        output='screen',
    )
    
    # Control node
    control_node = Node(
        package='object_following_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'max_linear_vel': 0.3,
            'max_angular_vel': 1.0,
            'target_distance': 0.5,
            'obstacle_threshold': 0.30,
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        target_class_arg,
        confidence_threshold_arg,
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        vision_node,
        bt_node,
        control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
    ])


