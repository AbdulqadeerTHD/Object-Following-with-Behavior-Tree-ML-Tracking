#!/usr/bin/env python3
"""
PID-based Control Node for Object Following Robot

This node:
- Subscribes to /object_position (from vision node)
- Subscribes to /scan (LiDAR for obstacle avoidance)
- Uses PID control to keep target centered and at safe distance
- Publishes /cmd_vel commands to robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from object_following_interfaces.msg import ObjectPosition
import numpy as np
import math


class PIDController:
    """Simple PID controller for angular and linear velocity control"""
    
    def __init__(self, kp, ki, kd, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute(self, error, dt):
        """
        Compute PID output
        
        Args:
            error: Current error value
            dt: Time step in seconds
            
        Returns:
            Output value
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.prev_error = 0.0


class ControlNode(Node):
    """ROS2 node for PID-based control of object following robot"""
    
    def __init__(self):
        super().__init__('control_node')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('target_distance', 0.5)  # Target distance in meters (based on y_size)
        self.declare_parameter('min_distance', 0.3)  # Minimum safe distance
        self.declare_parameter('obstacle_threshold', 0.4)  # LiDAR obstacle threshold in meters
        
        # PID parameters for angular control (centering)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.3)
        
        # PID parameters for linear control (distance)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.1)
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        
        # Create PID controllers
        angular_kp = self.get_parameter('angular_kp').get_parameter_value().double_value
        angular_ki = self.get_parameter('angular_ki').get_parameter_value().double_value
        angular_kd = self.get_parameter('angular_kd').get_parameter_value().double_value
        
        linear_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        linear_ki = self.get_parameter('linear_ki').get_parameter_value().double_value
        linear_kd = self.get_parameter('linear_kd').get_parameter_value().double_value
        
        self.angular_pid = PIDController(angular_kp, angular_ki, angular_kd, 
                                         max_output=self.max_angular_vel,
                                         min_output=-self.max_angular_vel)
        self.linear_pid = PIDController(linear_kp, linear_ki, linear_kd,
                                       max_output=self.max_linear_vel,
                                       min_output=0.0)  # Don't go backward
        
        # Store latest sensor data
        self.latest_object_position = None
        self.latest_lidar_scan = None
        self.current_behavior_state = "SEARCH"  # Per spec: Only FOLLOW and SEARCH
        self.last_control_time = self.get_clock().now()
        
        # Create subscribers
        self.object_sub = self.create_subscription(
            ObjectPosition,
            '/object_position',
            self.object_position_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Subscribe to behavior state from BT node (per spec: /behavior_state)
        self.state_sub = self.create_subscription(
            String,
            '/behavior_state',
            self.behavior_state_callback,
            10
        )
        
        # Create publisher to /cmd_vel (will be remapped to controller topic by launch file)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Also publish directly to controller topic as backup (for ROS2 Humble)
        # This ensures commands reach the controller even if remapping fails
        self.cmd_vel_controller_pub = self.create_publisher(
            Twist,
            '/diffdrive_controller/cmd_vel_unstamped',
            10
        )
        self.get_logger().info('Created backup publisher to /diffdrive_controller/cmd_vel_unstamped')
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Publish initial command immediately to activate controller
        initial_cmd = Twist()
        initial_cmd.linear.x = 0.0
        initial_cmd.angular.z = 0.3  # Start rotating in SEARCH mode
        self.cmd_vel_pub.publish(initial_cmd)
        self.cmd_vel_controller_pub.publish(initial_cmd)
        
        self.get_logger().info('Control Node initialized')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'Initial state: {self.current_behavior_state}')
        self.get_logger().info('Published initial rotation command - robot should start rotating')
    
    def object_position_callback(self, msg):
        """Callback for object position updates"""
        self.latest_object_position = msg
    
    def lidar_callback(self, msg):
        """Callback for LiDAR scan updates"""
        self.latest_lidar_scan = msg
    
    def behavior_state_callback(self, msg):
        """Callback for behavior state updates from BT node (per spec: /behavior_state)"""
        old_state = self.current_behavior_state
        self.current_behavior_state = msg.data
        if old_state != self.current_behavior_state:
            self.get_logger().info(f'Behavior state changed: {old_state} → {self.current_behavior_state}')
    
    def check_obstacle(self):
        """
        Check for obstacles using LiDAR data
        
        Returns:
            tuple: (has_obstacle, obstacle_direction)
                   obstacle_direction: 'left', 'right', 'front', or None
        """
        if self.latest_lidar_scan is None:
            return False, None
        
        ranges = np.array(self.latest_lidar_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return False, None
        
        # Divide scan into sectors
        num_ranges = len(ranges)
        front_start = int(num_ranges * 0.4)
        front_end = int(num_ranges * 0.6)
        left_start = int(num_ranges * 0.7)
        left_end = num_ranges
        right_start = 0
        right_end = int(num_ranges * 0.3)
        
        # Get minimum distances
        front_ranges = ranges[front_start:front_end]
        left_ranges = ranges[left_start:left_end]
        right_ranges = ranges[right_start:right_end]
        
        # Filter out invalid values first, then check if array is not empty
        front_valid = front_ranges[np.isfinite(front_ranges)]
        left_valid = left_ranges[np.isfinite(left_ranges)]
        right_valid = right_ranges[np.isfinite(right_ranges)]
        
        min_front = np.min(front_valid) if len(front_valid) > 0 else float('inf')
        min_left = np.min(left_valid) if len(left_valid) > 0 else float('inf')
        min_right = np.min(right_valid) if len(right_valid) > 0 else float('inf')
        
        # Check for obstacles
        if min_front < self.obstacle_threshold:
            return True, 'front'
        elif min_left < self.obstacle_threshold:
            return True, 'left'
        elif min_right < self.obstacle_threshold:
            return True, 'right'
        
        return False, None
    
    def control_loop(self):
        """
        Main control loop - called periodically
        
        Per specification:
        - FOLLOW MODE: Simple control logic (non-PID)
          if target_x < 0.4 → turn left
          if target_x > 0.6 → turn right
          if 0.4 <= target_x <= 0.6 → go forward
        - SEARCH MODE: Robot rotates slowly until object detected
        - Obstacle Avoidance: If LiDAR < 0.3m → STOP or slight turn
        """
        cmd_vel = Twist()
        
        # Check for obstacles first (per spec: override FOLLOW mode when danger detected)
        has_obstacle, obstacle_dir = self.check_obstacle()
        
        if has_obstacle:
            # Per spec: If any reading < 0.3m → STOP or slight turn
            cmd_vel.linear.x = 0.0  # STOP
            if obstacle_dir == 'front':
                # Slight turn away from obstacle
                if self.latest_lidar_scan:
                    ranges = np.array(self.latest_lidar_scan.ranges)
                    num_ranges = len(ranges)
                    left_avg = np.mean(ranges[int(num_ranges * 0.7):][np.isfinite(ranges[int(num_ranges * 0.7):])])
                    right_avg = np.mean(ranges[:int(num_ranges * 0.3)][np.isfinite(ranges[:int(num_ranges * 0.3)])])
                    if left_avg > right_avg:
                        cmd_vel.angular.z = -0.3  # Slight turn right
                    else:
                        cmd_vel.angular.z = 0.3   # Slight turn left
            elif obstacle_dir == 'left':
                cmd_vel.angular.z = -0.3  # Turn right
            elif obstacle_dir == 'right':
                cmd_vel.angular.z = 0.3   # Turn left
        else:
            # Behavior tree based control (per spec: only FOLLOW and SEARCH)
            if self.current_behavior_state == "FOLLOW":
                # FOLLOW MODE: Per spec - simple control logic (non-PID)
                if self.latest_object_position is not None and self.latest_object_position.found:
                    # Get normalized x position (0.0 to 1.0)
                    target_x = self.latest_object_position.x
                    
                    # Per spec: Simple control logic
                    if target_x < 0.4:
                        # Turn left
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.3
                    elif target_x > 0.6:
                        # Turn right
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = -0.3
                    else:
                        # 0.4 <= target_x <= 0.6 → go forward
                        # Adjust speed based on distance (closer = slower)
                        if hasattr(self.latest_object_position, 'distance') and self.latest_object_position.distance > 0:
                            # Use distance estimate if available
                            if self.latest_object_position.distance < 0.3:
                                cmd_vel.linear.x = 0.0  # Stop if too close
                            elif self.latest_object_position.distance < 0.5:
                                cmd_vel.linear.x = 0.1  # Slow if close
                            else:
                                cmd_vel.linear.x = self.max_linear_vel  # Normal speed
                        else:
                            # Fallback: use distance estimate (smaller distance = closer)
                            if hasattr(self.latest_object_position, 'distance') and self.latest_object_position.distance > 0:
                                if self.latest_object_position.distance < 0.3:
                                    cmd_vel.linear.x = 0.0  # Stop if too close
                                elif self.latest_object_position.distance < 0.5:
                                    cmd_vel.linear.x = 0.1  # Slow if close
                                else:
                                    cmd_vel.linear.x = self.max_linear_vel  # Normal speed
                            else:
                                cmd_vel.linear.x = self.max_linear_vel  # Default forward
                        cmd_vel.angular.z = 0.0
                else:
                    # Object lost during FOLLOW - stop
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
            
            elif self.current_behavior_state == "SEARCH":
                # SEARCH MODE: Per spec - robot rotates slowly until object detected
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.3  # Slow rotation (per spec)
                # Log occasionally to verify SEARCH mode is active
                if not hasattr(self, '_search_log_count'):
                    self._search_log_count = 0
                self._search_log_count += 1
                if self._search_log_count % 20 == 0:  # Every 1 second at 20Hz
                    self.get_logger().info(f'SEARCH mode: Rotating at {cmd_vel.angular.z} rad/s')
            
            else:
                # Unknown state - default to SEARCH (rotate)
                self.get_logger().warn(f'Unknown state: {self.current_behavior_state}, defaulting to SEARCH')
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.3  # Rotate in unknown state too
        
        # ALWAYS publish command (even if zero) to keep controller active
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Also publish to controller topic directly as backup (ensures controller receives commands)
        self.cmd_vel_controller_pub.publish(cmd_vel)
        
        # Log occasionally to verify commands are being published
        if not hasattr(self, '_publish_log_count'):
            self._publish_log_count = 0
        self._publish_log_count += 1
        if self._publish_log_count % 100 == 0:  # Every 5 seconds at 20Hz
            self.get_logger().info(f'Published cmd_vel: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}, state={self.current_behavior_state}')
    
    def publish_stop(self):
        """Publish stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

