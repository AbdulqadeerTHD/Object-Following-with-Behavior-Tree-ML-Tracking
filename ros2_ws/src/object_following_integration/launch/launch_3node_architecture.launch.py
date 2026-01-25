"""
Launch file for 3-Node Architecture:
1. Vision Node (YOLOv8 + SORT) - publishes /detected_persons
2. Behavior Manager Node - SEARCH/FOLLOW/WAIT state machine - publishes /behavior_cmd
3. Controller Node - converts behavior_cmd to cmd_vel with velocity clamping
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
    
    # Get world file path
    world_file = os.path.join(integration_package_dir, 'worlds', 'moving_world.wbt')
    
    # Launch Webots
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
    
    # ROS2 control parameters
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
    
    # NODE 1: Vision YOLO Node (YOLOv8 + SORT)
    vision_yolo_node = Node(
        package='object_following_vision',
        executable='vision_yolo',
        name='vision_yolo_node',
        output='screen',
    )
    
    # NODE 2: Behavior Manager Node (State Machine)
    behavior_manager_node = Node(
        package='object_following_bt',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
    )
    
    # NODE 3: Controller Node (cmd_vel with clamping)
    controller_node = Node(
        package='object_following_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        vision_yolo_node,      # Node 1: Vision (YOLOv8 + SORT)
        behavior_manager_node,  # Node 2: Behavior Manager
        controller_node,        # Node 3: Controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
    ])


