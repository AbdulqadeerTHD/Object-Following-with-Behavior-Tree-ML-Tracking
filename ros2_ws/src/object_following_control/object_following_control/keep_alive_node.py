#!/usr/bin/env python3
"""
Keep Alive Node - Publishes zero commands to keep controller active

This allows teleop to work by keeping the controller subscribed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeepAliveNode(Node):
    def __init__(self):
        super().__init__('keep_alive_node')
        # Use QoS profile that ensures delivery
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # Publish to /cmd_vel which gets remapped to controller topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.get_logger().info('Keep alive node started - publishing zero commands at low rate')
        # Publish immediately and then start timer at low rate (1Hz) so teleop can override
        self.publish_zero()
        self.timer = self.create_timer(1.0, self.publish_zero)  # 1Hz - just to keep controller alive
    
    def publish_zero(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = KeepAliveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

