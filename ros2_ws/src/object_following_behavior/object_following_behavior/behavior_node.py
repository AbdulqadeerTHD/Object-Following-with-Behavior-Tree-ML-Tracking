#!/usr/bin/env python3
"""
Behavior Tree Node for Object Following Robot

This node manages high-level robot states:
- DETECT: Initial state, waiting for object detection
- FOLLOW: Object detected, following mode
- SEARCH: Object lost, searching for target
- DELIVER: (Future) Delivery mode

States are published on /system_state topic for coordination with other nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from object_following_vision.msg import ObjectPosition
from enum import Enum
import time


class RobotState(Enum):
    """Robot behavior states"""
    DETECT = "DETECT"
    FOLLOW = "FOLLOW"
    SEARCH = "SEARCH"
    DELIVER = "DELIVER"
    RETURN = "RETURN"


class BehaviorNode(Node):
    """ROS2 node for behavior tree and state management"""
    
    def __init__(self):
        super().__init__('behavior_node')
        
        # Declare parameters
        self.declare_parameter('search_timeout', 5.0)  # Seconds to wait before switching to SEARCH
        self.declare_parameter('detection_timeout', 2.0)  # Seconds to wait before confirming detection
        self.declare_parameter('delivery_timeout', 10.0)  # Seconds to stay in DELIVER mode
        self.declare_parameter('return_enabled', False)  # Enable RETURN state
        
        # Get parameters
        self.search_timeout = self.get_parameter('search_timeout').get_parameter_value().double_value
        self.detection_timeout = self.get_parameter('detection_timeout').get_parameter_value().double_value
        self.delivery_timeout = self.get_parameter('delivery_timeout').get_parameter_value().double_value
        self.return_enabled = self.get_parameter('return_enabled').get_parameter_value().bool_value
        
        # State management
        self.current_state = RobotState.DETECT
        self.last_detection_time = None
        self.last_lost_time = None
        self.detection_count = 0
        self.delivery_start_time = None
        self.start_position = None  # For RETURN state
        self.following_duration = 0.0
        self.last_follow_time = None
        
        # Create subscriber for object position
        self.object_sub = self.create_subscription(
            ObjectPosition,
            '/object_position',
            self.object_position_callback,
            10
        )
        
        # Create publisher for system state
        self.state_pub = self.create_publisher(
            String,
            '/system_state',
            10
        )
        
        # State update timer (10 Hz)
        self.state_timer = self.create_timer(0.1, self.update_state)
        
        # Publish initial state
        self.publish_state()
        
        self.get_logger().info('Behavior Node initialized')
        self.get_logger().info(f'Initial state: {self.current_state.value}')
        self.get_logger().info(f'Search timeout: {self.search_timeout} seconds')
    
    def object_position_callback(self, msg):
        """Callback for object position updates"""
        current_time = time.time()
        
        if msg.detected:
            self.last_detection_time = current_time
            self.detection_count += 1
        else:
            if self.last_detection_time is not None:
                # Object was detected before, now lost
                if self.last_lost_time is None:
                    self.last_lost_time = current_time
            else:
                # Never detected, update lost time
                self.last_lost_time = current_time
    
    def update_state(self):
        """Update robot state based on object detection status"""
        current_time = time.time()
        new_state = self.current_state
        
        # State machine logic
        if self.current_state == RobotState.DETECT:
            # Waiting for initial detection
            if self.last_detection_time is not None:
                time_since_detection = current_time - self.last_detection_time
                # Switch to FOLLOW immediately when object detected (no timeout needed)
                if time_since_detection < 5.0:  # Object detected within last 5 seconds
                    new_state = RobotState.FOLLOW
                    self.get_logger().info('Object detected! Switching to FOLLOW mode')
                else:
                    # Detection timeout, switch to SEARCH
                    new_state = RobotState.SEARCH
                    self.get_logger().info('Detection timeout, switching to SEARCH mode')
            else:
                # No detection yet, check if we should switch to SEARCH
                if self.last_lost_time is not None:
                    time_since_lost = current_time - self.last_lost_time
                    if time_since_lost > self.search_timeout:
                        new_state = RobotState.SEARCH
                        self.get_logger().info('No detection found, switching to SEARCH mode')
        
        elif self.current_state == RobotState.FOLLOW:
            # Following mode
            if self.last_detection_time is not None:
                # Object detected, track following duration
                if self.last_follow_time is None:
                    self.last_follow_time = current_time
                self.following_duration = current_time - self.last_follow_time
                self.last_lost_time = None
            else:
                # Object lost
                if self.last_lost_time is None:
                    self.last_lost_time = current_time
                self.last_follow_time = None
                
                time_since_lost = current_time - self.last_lost_time
                if time_since_lost >= self.search_timeout:
                    # Lost for too long, switch to SEARCH
                    new_state = RobotState.SEARCH
                    self.get_logger().warn(f'Object lost for {time_since_lost:.1f}s. Switching to SEARCH mode')
        
        elif self.current_state == RobotState.SEARCH:
            # Searching for object
            if self.last_detection_time is not None:
                time_since_detection = current_time - self.last_detection_time
                if time_since_detection < self.detection_timeout:
                    # Object found! Switch to FOLLOW
                    new_state = RobotState.FOLLOW
                    self.get_logger().info('Object found! Switching to FOLLOW mode')
                    self.last_lost_time = None
        
        elif self.current_state == RobotState.DELIVER:
            # Delivery mode: Stay in delivery for specified time, then RETURN or SEARCH
            if self.delivery_start_time is None:
                self.delivery_start_time = current_time
            
            delivery_duration = current_time - self.delivery_start_time
            
            if delivery_duration >= self.delivery_timeout:
                # Delivery complete, switch to RETURN or SEARCH
                if self.return_enabled:
                    new_state = RobotState.RETURN
                    self.get_logger().info('Delivery complete! Switching to RETURN mode')
                else:
                    new_state = RobotState.SEARCH
                    self.get_logger().info('Delivery complete! Switching to SEARCH mode')
                self.delivery_start_time = None
        
        elif self.current_state == RobotState.RETURN:
            # Return mode: Return to start position
            # For now, switch to SEARCH if object detected (can resume following)
            if self.last_detection_time is not None:
                time_since_detection = current_time - self.last_detection_time
                if time_since_detection < self.detection_timeout:
                    # Object found during return, can resume following
                    new_state = RobotState.FOLLOW
                    self.get_logger().info('Object found during return! Resuming FOLLOW mode')
        
        # Update state if changed
        if new_state != self.current_state:
            self.current_state = new_state
            self.publish_state()
    
    def publish_state(self):
        """Publish current system state"""
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
        self.get_logger().debug(f'Published state: {self.current_state.value}')
    
    def get_state(self):
        """Get current robot state"""
        return self.current_state.value


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = BehaviorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

