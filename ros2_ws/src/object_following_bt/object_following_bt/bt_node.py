#!/usr/bin/env python3
"""
Behavior Tree Node for Object Following Robot

Per specification:
- Implements CheckObjectVisible condition
- Outputs FOLLOW or SEARCH state
- Publishes to /behavior_state

BT Structure:
ROOT
├── CheckObjectVisible (reads found flag from /object_position)
│   ├── SUCCESS → FOLLOW branch
│   └── FAILURE → SEARCH branch
│
├── FOLLOW
│   └── Output state "FOLLOW"
│
└── SEARCH
    └── Output state "SEARCH"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from object_following_interfaces.msg import ObjectPosition


class CheckObjectVisible:
    """BT Condition: Check if object is visible"""
    
    def __init__(self, object_position_msg):
        self.object_position_msg = object_position_msg
    
    def tick(self):
        """Return SUCCESS if object found, FAILURE otherwise"""
        if self.object_position_msg is not None and self.object_position_msg.found:
            return "SUCCESS"
        return "FAILURE"


class FollowAction:
    """BT Action: Follow mode"""
    
    def __init__(self, node):
        self.node = node
    
    def tick(self):
        """Execute FOLLOW action"""
        return "SUCCESS"


class SearchAction:
    """BT Action: Search mode"""
    
    def __init__(self, node):
        self.node = node
    
    def tick(self):
        """Execute SEARCH action"""
        return "SUCCESS"


class BehaviorTreeNode(Node):
    """ROS2 Behavior Tree Node for object following robot"""
    
    def __init__(self):
        super().__init__('bt_node')
        
        # Current object position (from vision node)
        self.current_object_position = None
        
        # Create subscriber for object position
        self.object_sub = self.create_subscription(
            ObjectPosition,
            '/object_position',
            self.object_position_callback,
            10
        )
        
        # Create publisher for behavior state
        self.state_pub = self.create_publisher(
            String,
            '/behavior_state',
            10
        )
        
        # Initialize BT components
        self.check_visible = CheckObjectVisible(None)
        self.follow_action = FollowAction(self)
        self.search_action = SearchAction(self)
        
        # Current state
        self.current_state = "SEARCH"  # Start in SEARCH mode
        
        # Behavior tree execution timer (10 Hz)
        self.bt_timer = self.create_timer(0.1, self.execute_behavior_tree)
        
        # Publish initial state immediately and repeatedly
        self.publish_state("SEARCH")
        
        self.get_logger().info('Behavior Tree Node initialized')
        self.get_logger().info('Initial state: SEARCH')
        self.get_logger().info('Publishing to: /behavior_state')
        self.get_logger().info('Subscribed to: /object_position')
        self.get_logger().info('BT will publish SEARCH state continuously until object detected')
    
    def object_position_callback(self, msg):
        """Callback for object position updates from vision node"""
        self.current_object_position = msg
        # Update the condition with latest message
        self.check_visible.object_position_msg = msg
    
    def execute_behavior_tree(self):
        """
        Execute Behavior Tree logic:
        
        ROOT
        ├── CheckObjectVisible
        │   ├── SUCCESS → FOLLOW
        │   └── FAILURE → SEARCH
        """
        # Update condition with current object position
        self.check_visible.object_position_msg = self.current_object_position
        
        # Execute CheckObjectVisible condition
        condition_result = self.check_visible.tick()
        
        # Execute appropriate action based on condition
        if condition_result == "SUCCESS":
            # Object is visible → FOLLOW mode
            if self.current_state != "FOLLOW":
                self.get_logger().info('Object detected → Switching to FOLLOW mode')
                self.current_state = "FOLLOW"
            self.follow_action.tick()
            self.publish_state("FOLLOW")
        else:
            # Object not visible → SEARCH mode
            if self.current_state != "SEARCH":
                self.get_logger().info('Object lost → Switching to SEARCH mode')
                self.current_state = "SEARCH"
            self.search_action.tick()
            self.publish_state("SEARCH")
    
    def publish_state(self, state):
        """Publish behavior state to /behavior_state topic"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        # Log state changes
        if not hasattr(self, '_last_published_state') or self._last_published_state != state:
            self.get_logger().info(f'Published behavior state: {state}')
            self._last_published_state = state


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = BehaviorTreeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
