"""
Launch file for TRAINING: Manual control + Data Collection + Autonomous Behavior
- You manually move the robot and the system captures images
- Robot learns to detect and approach pedestrians
- Autonomous behavior: SEARCH → DETECT → APPROACH → STOP → SEARCH
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # Get package directories
    integration_package_dir = get_package_share_directory('object_following_integration')
    turtlebot_package_dir = get_package_share_directory('webots_ros2_turtlebot')
    
    # Get world file path - TRAINING WORLD (fresh, clean environment)
    world_file = os.path.join(integration_package_dir, 'worlds', 'training_world.wbt')
    
    # Launch Webots with training world
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
    
    # Webots controller
    custom_urdf_installed = os.path.join(integration_package_dir, 'resource', 'turtlebot_webots_with_camera.urdf')
    default_urdf_path = os.path.join(turtlebot_package_dir, 'resource', 'turtlebot_webots.urdf')
    robot_description_path = custom_urdf_installed if os.path.exists(custom_urdf_installed) else default_urdf_path
    
    # ROS2 control parameters - use custom config with max speed
    custom_ros2_control = os.path.join(integration_package_dir, 'resource', 'ros2control.yml')
    ros2_control_params = custom_ros2_control if os.path.exists(custom_ros2_control) else os.path.join(turtlebot_package_dir, 'resource', 'ros2control.yml')
    
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
    
    # DATA COLLECTION NODE - Automatically save camera images
    data_collector = Node(
        package='object_following_vision',
        executable='data_collector',
        name='data_collector',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'save_interval': 5,  # Save every 5 frames
            'output_dir': 'dataset/images',
            'max_images': 1000  # Collect up to 1000 images
        }]
    )
    
    # VISION NODE - Detect pedestrians (persons)
    vision_node = Node(
        package='object_following_vision',
        executable='yolov8_tracker',
        name='vision_node',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'target_class_name': 'person',  # Detect persons/pedestrians
            'confidence_threshold': 0.3
        }]
    )
    
    # BEHAVIOR TREE NODE - State machine (SEARCH, DETECT, APPROACH, STOP)
    bt_node = Node(
        package='object_following_bt',
        executable='bt_node',
        name='bt_node',
        output='screen',
    )
    
    # CONTROL NODE - Movement control
    control_node = Node(
        package='object_following_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'max_linear_vel': 2.0,
            'max_angular_vel': 4.0,
            'target_distance': 0.5,
            'obstacle_threshold': 0.30,
        }]
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        data_collector,  # Automatically saves images
        vision_node,     # Detects pedestrians
        bt_node,         # State machine
        control_node,    # Movement control
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
    ])

