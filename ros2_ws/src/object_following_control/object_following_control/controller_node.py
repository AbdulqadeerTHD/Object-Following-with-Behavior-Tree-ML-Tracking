#!/usr/bin/env python3
"""
Controller Node: Converts behavior commands to cmd_vel
Includes velocity clamping and obstacle avoidance

This node translates high-level behavior commands into low-level motor commands:
- ROTATE: Rotates in place (0.8 rad/s, no forward movement)
- STOP: Stops completely
- FOLLOW: Moves toward person with obstacle avoidance

Obstacle Avoidance:
- Uses LiDAR to detect obstacles in front sector (30-70% of scan)
- Obstacle threshold: 0.40m (40cm safety distance)
- Bypass strategy: Turns left/right based on clearer path while moving forward
- Clear threshold: 0.60m (considers obstacle cleared)

Velocity Control:
- Maximum linear speed: 0.22 m/s (TurtleBot3 Burger limit)
- Maximum angular speed: 2.0 rad/s
- Proportional control for person following (angular velocity based on x_center error)
- Speed adjustment based on person area (fill ratio)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import numpy as np


class ControllerNode(Node):
    """ROS2 node that converts behavior commands to motion commands"""
    
    def __init__(self):
        super().__init__('controller_node')
        
        # LiDAR data for obstacle avoidance
        self.latest_lidar_scan = None
        self.obstacle_threshold = 0.40  # 40cm safety threshold
        self.obstacle_clear_threshold = 0.60  # 60cm - clear of obstacle
        self.bypass_direction = None  # Remember which direction we're turning to bypass
        self.obstacle_detection_count = 0  # Count consecutive obstacle detections
        
        # Subscribe to behavior commands
        self.sub = self.create_subscription(
            String,
            '/behavior_cmd',
            self.cmd_callback,
            10
        )
        self.get_logger().info('Subscribed to /behavior_cmd')
        
        # Subscribe to LiDAR for obstacle avoidance
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.get_logger().info('Subscribed to /scan (LiDAR)')
        
        # Publish cmd_vel
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info('Publishing to /cmd_vel')
        
        self.get_logger().info('Controller Node initialized')

        # Logging counter
        self._cmd_log_count = 0
    
    def lidar_callback(self, msg):
        """Callback for LiDAR scan data"""
        self.latest_lidar_scan = msg
        # Log occasionally to verify LiDAR is working
        if not hasattr(self, '_lidar_log_count'):
            self._lidar_log_count = 0
        self._lidar_log_count += 1
        if self._lidar_log_count % 100 == 0:  # Log every 10 seconds
            ranges = np.array(msg.ranges)
            valid_ranges = ranges[np.isfinite(ranges)]
            if len(valid_ranges) > 0:
                min_range = np.min(valid_ranges)
                max_range = np.max(valid_ranges)
                avg_range = np.mean(valid_ranges)
                self.get_logger().info(f'LiDAR: {len(valid_ranges)}/{len(ranges)} valid ranges, min={min_range:.2f}m, max={max_range:.2f}m, avg={avg_range:.2f}m')
    
    def check_obstacle(self):
        """Check for obstacles using LiDAR - improved detection"""
        if self.latest_lidar_scan is None:
            return False
        
        ranges = np.array(self.latest_lidar_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return False
        
        # Check wider front sector (30% to 70% of scan) - covers ~144 degrees
        # LiDAR scans 360 degrees, so front is roughly center of array
        num_ranges = len(ranges)
        front_start = int(num_ranges * 0.3)
        front_end = int(num_ranges * 0.7)
        front_ranges = ranges[front_start:front_end]
        front_valid = front_ranges[np.isfinite(front_ranges)]
        
        if len(front_valid) > 0:
            min_front = np.min(front_valid)
            if min_front < self.obstacle_threshold:
                self.obstacle_detection_count += 1
                return True
        
        # If we were bypassing and now clear, reset
        if self.bypass_direction is not None:
            if len(front_valid) > 0:
                min_front = np.min(front_valid)
                if min_front > self.obstacle_clear_threshold:
                    # Clear of obstacle, reset bypass state
                    self.bypass_direction = None
                    self.obstacle_detection_count = 0
        
        return False
    
    def cmd_callback(self, msg):
        """Process behavior commands and convert to cmd_vel"""
        cmd = Twist()
        
        # Process behavior command first - ROTATE and STOP don't check obstacles
        if msg.data == "ROTATE":
            # Rotate in place to search - NO obstacle check, just rotate
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8  # Rotate faster to search for person
        
        elif msg.data == "STOP":
            # Stop completely
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        else:
            # FOLLOW command (JSON format) - check for obstacles during following
            try:
                data = json.loads(msg.data)
                
                if data.get("cmd") == "FOLLOW":
                    # Check for obstacles first (only during FOLLOW mode)
                    has_obstacle = self.check_obstacle()
                    
                    if has_obstacle:
                        # Obstacle detected - turn and continue forward to bypass it
                        # Keep moving forward while turning to go around obstacle
                        if self.latest_lidar_scan:
                            ranges = np.array(self.latest_lidar_scan.ranges)
                            num_ranges = len(ranges)
                            
                            # Check left and right sectors more carefully
                            # Left sector: 60% to 100% (right side of robot)
                            # Right sector: 0% to 40% (left side of robot)
                            left_sector = ranges[int(num_ranges * 0.6):]
                            right_sector = ranges[:int(num_ranges * 0.4)]
                            left_valid = left_sector[np.isfinite(left_sector)]
                            right_valid = right_sector[np.isfinite(right_sector)]
                            
                            # If we're already bypassing, continue in same direction
                            if self.bypass_direction is not None:
                                if self.bypass_direction == "left":
                                    cmd.angular.z = 1.5  # Continue turning left
                                    cmd.linear.x = 0.20
                                else:  # right
                                    cmd.angular.z = -1.5  # Continue turning right
                                    cmd.linear.x = 0.20
                            else:
                                # Decide which direction to turn
                                if len(left_valid) > 0 and len(right_valid) > 0:
                                    left_avg = np.mean(left_valid)
                                    right_avg = np.mean(right_valid)
                                    left_min = np.min(left_valid)
                                    right_min = np.min(right_valid)
                                    
                                    # Choose direction with more clearance
                                    if left_avg > right_avg and left_min > right_min:
                                        # Left side clearer - turn right (negative angular)
                                        self.bypass_direction = "right"
                                        cmd.angular.z = -1.5  # Turn right to bypass
                                        cmd.linear.x = 0.20
                                    else:
                                        # Right side clearer - turn left (positive angular)
                                        self.bypass_direction = "left"
                                        cmd.angular.z = 1.5   # Turn left to bypass
                                        cmd.linear.x = 0.20
                                else:
                                    # Default: turn left and continue forward
                                    self.bypass_direction = "left"
                                    cmd.angular.z = 1.5
                                    cmd.linear.x = 0.20
                        else:
                            # Default: turn left and continue forward
                            if self.bypass_direction is None:
                                self.bypass_direction = "left"
                            cmd.angular.z = 1.5 if self.bypass_direction == "left" else -1.5
                            cmd.linear.x = 0.20
                        
                        if not hasattr(self, '_obstacle_log_count'):
                            self._obstacle_log_count = 0
                        self._obstacle_log_count += 1
                        if self._obstacle_log_count % 10 == 0:  # Log more frequently
                            direction = self.bypass_direction or "deciding"
                            self.get_logger().warn(f'Obstacle detected (count={self.obstacle_detection_count}) - bypassing {direction} (linear={cmd.linear.x:.2f} m/s, angular={cmd.angular.z:.2f} rad/s)')
                    else:
                        # No obstacle - follow normally
                        # Calculate error from center (assuming 640px width)
                        x_center = data.get("x", 320)
                        error = x_center - 320
                        
                        # Angular velocity proportional to error (very responsive)
                        # Use stronger gain for faster, more accurate turning
                        # Negative because: if person is to the right (x > 320), turn right (negative angular)
                        cmd.angular.z = -0.012 * error  # Increased gain for faster, more accurate turning
                        
                        # Limit angular velocity to prevent overshooting
                        if abs(cmd.angular.z) > 1.8:
                            cmd.angular.z = 1.8 if cmd.angular.z > 0 else -1.8
                        
                        # Linear velocity based on area
                        # Person detected at ~307200 when filling entire frame (640x480)
                        # ALWAYS move forward aggressively - keep moving forward until very close
                        area = data.get("area", 0)
                        max_area = 307200  # 640x480 = full frame
                        
                        # Calculate how much of frame person fills (0.0 to 1.0)
                        fill_ratio = area / max_area if max_area > 0 else 0.0
                        
                        # ALWAYS move forward at maximum or high speed - follow aggressively
                        # Only slow down when extremely close (almost filling entire frame)
                        if fill_ratio > 0.998:
                            # Person fills >99.8% of frame - extremely close, slow down slightly but keep moving
                            cmd.linear.x = 0.18  # Still move forward (almost at target)
                        elif fill_ratio > 0.99:
                            # Person fills 99-99.8% - very close, keep moving forward fast
                            cmd.linear.x = 0.22  # Maximum forward speed (keep moving forward)
                        elif fill_ratio > 0.95:
                            # Person fills 95-99% - close, maximum speed
                            cmd.linear.x = 0.22  # Maximum forward speed
                        elif fill_ratio > 0.80:
                            # Person fills 80-95% - medium distance, maximum speed
                            cmd.linear.x = 0.22  # Maximum forward speed
                        else:
                            # Person fills <80% - far away, maximum speed
                            cmd.linear.x = 0.22  # Maximum forward speed
                        
                        # Log occasionally
                        if not hasattr(self, '_follow_log_count'):
                            self._follow_log_count = 0
                        self._follow_log_count += 1
                        if self._follow_log_count % 20 == 0:  # Log every 2 seconds
                            self.get_logger().info(f'FOLLOW: x={x_center:.1f}, area={area:.0f}, linear={cmd.linear.x:.2f} m/s, angular={cmd.angular.z:.3f} rad/s')
            
            except (json.JSONDecodeError, KeyError) as e:
                self.get_logger().error(f'Error parsing behavior command: {e}')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        # IMPORTANT: Clamp velocities to prevent warnings
        # Max linear: 0.22 m/s (TurtleBot3 Burger max speed)
        cmd.linear.x = max(min(cmd.linear.x, 0.22), -0.22)
        # Max angular: 2.0 rad/s
        cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0)
        
        # Log published command occasionally
        self._cmd_log_count += 1
        if self._cmd_log_count % 20 == 0:  # Log every 2 seconds
            self.get_logger().info(f'CMD_VEL: linear={cmd.linear.x:.2f} m/s, angular={cmd.angular.z:.3f} rad/s')

        # Publish command
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Already shut down


if __name__ == '__main__':
    main()
