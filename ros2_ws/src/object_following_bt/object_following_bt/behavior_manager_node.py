#!/usr/bin/env python3
"""
Behavior Manager Node: State Machine
SEARCH → FOLLOW → WAIT → SEARCH
Manages person IDs and avoids re-following completed persons
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class BehaviorManager(Node):
    """ROS2 node that manages robot behavior states"""
    
    def __init__(self):
        super().__init__('behavior_manager')
        
        # State machine
        self.state = "SEARCH"
        self.current_id = None
        self.completed_ids = set()  # Track completed person IDs
        self.last_persons = []
        self.wait_start = None
        self.follow_start = None  # Track when FOLLOW mode started
        self.last_detection_time = None
        
        # Subscribe to detected persons
        self.sub = self.create_subscription(
            String,
            '/detected_persons',
            self.person_callback,
            10
        )
        self.get_logger().info('Subscribed to /detected_persons')
        
        # Publish behavior commands
        self.pub = self.create_publisher(
            String,
            '/behavior_cmd',
            10
        )
        self.get_logger().info('Publishing to /behavior_cmd')
        
        # Main loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.loop)
        
        # Timer to check if receiving detections
        self.check_timer = self.create_timer(5.0, self.check_detections)
        self.last_detection_time = None
        
        self.get_logger().info('Behavior Manager Node initialized')
        self.get_logger().info(f'Initial state: {self.state}')
    
    def check_detections(self):
        """Check if we're receiving detections"""
        if self.last_detection_time is None:
            self.get_logger().warn('No detections received from /detected_persons yet!')
            self.get_logger().warn('Check: ros2 topic echo /detected_persons --once')
        elif len(self.last_persons) == 0:
            self.get_logger().warn(f'Receiving empty detections (no persons found)')
    
    def person_callback(self, msg):
        """Callback for detected persons"""
        try:
            self.last_persons = json.loads(msg.data)
            self.last_detection_time = self.get_clock().now()
            
            if len(self.last_persons) > 0:
                if not hasattr(self, '_last_log_count'):
                    self._last_log_count = 0
                self._last_log_count += 1
                # Always log when persons are detected
                self.get_logger().info(f'Received {len(self.last_persons)} person(s): IDs {[p["id"] for p in self.last_persons]}')
            else:
                if not hasattr(self, '_empty_count'):
                    self._empty_count = 0
                self._empty_count += 1
                if self._empty_count % 50 == 0:
                    self.get_logger().warn(f'Received empty detections ({self._empty_count} times) - no persons detected')
        except Exception as e:
            self.get_logger().error(f'Error parsing persons data: {e}')
            self.last_persons = []
    
    def loop(self):
        """Main behavior loop"""
        if self.state == "SEARCH":
            # Search for new person (not in completed_ids)
            if self.last_persons and len(self.last_persons) > 0:
                for p in self.last_persons:
                    person_id = p.get("id")
                    if person_id is not None and person_id not in self.completed_ids:
                        # Found new person - switch to FOLLOW
                        self.current_id = person_id
                        self.state = "FOLLOW"
                        self.follow_start = time.time()  # Record when FOLLOW started
                        area = p.get("area", 0)
                        x_center = p.get("x_center", 320)
                        self.get_logger().info(f'Person {self.current_id} detected (area={area:.0f}, x={x_center:.1f}) -> Switching to FOLLOW mode')
                        # Immediately publish FOLLOW command
                        cmd = String()
                        cmd.data = json.dumps({
                            "cmd": "FOLLOW",
                            "x": x_center,
                            "area": area
                        })
                        self.pub.publish(cmd)
                        self.get_logger().info(f'Published FOLLOW command for Person {self.current_id}')
                        return  # Exit early to process FOLLOW state
            
            # No new person found - rotate continuously
            if self.state == "SEARCH":
                cmd = String()
                cmd.data = "ROTATE"
                self.pub.publish(cmd)
                # Log occasionally
                if not hasattr(self, '_search_log_count'):
                    self._search_log_count = 0
                self._search_log_count += 1
                if self._search_log_count % 50 == 0:  # Every 5 seconds
                    if len(self.last_persons) > 0:
                        self.get_logger().warn(f'SEARCH mode: Person detected but already completed? IDs: {[p.get("id") for p in self.last_persons]}, Completed: {self.completed_ids}')
                    else:
                        self.get_logger().info(f'SEARCH mode: Rotating, waiting for person detection...')
        
        elif self.state == "FOLLOW":
            # Find target person by ID
            target = None
            if self.last_persons:
                for p in self.last_persons:
                    if p.get("id") == self.current_id:
                        target = p
                        break
            
            if target:
                # Publish FOLLOW command with target info
                cmd = String()
                cmd.data = json.dumps({
                    "cmd": "FOLLOW",
                    "x": target.get("x_center", 320),
                    "area": target.get("area", 0)
                })
                self.pub.publish(cmd)
                
                # Log occasionally
                if not hasattr(self, '_follow_log_count'):
                    self._follow_log_count = 0
                self._follow_log_count += 1
                if self._follow_log_count % 20 == 0:  # Log more frequently (every 2 seconds)
                    area = target.get("area", 0)
                    x_center = target.get("x_center", 320)
                    self.get_logger().info(f'FOLLOW mode: Person {self.current_id}, area={area:.0f}, x={x_center:.1f} (publishing commands)')
                
                # Check if close enough
                # Frame is 640x480 = 307200 pixels max
                # Only switch to WAIT if area is at maximum (307200) consistently
                # This means person fills entire frame - robot is very close
                area = target.get("area", 0)
                follow_duration = time.time() - self.follow_start if self.follow_start else 0
                
                # Track area history to detect if we're at maximum consistently
                if not hasattr(self, '_area_history'):
                    self._area_history = []
                self._area_history.append(area)
                if len(self._area_history) > 100:  # Keep last 100 measurements (~10 seconds)
                    self._area_history.pop(0)
                
                # Check if area is consistently very high (close to 307200) - person fills most of frame
                # Frame is 640x480 = 307200 pixels max
                # Consider "very close" when area > 306500 (99.8% of frame) consistently
                recent_areas = self._area_history[-30:] if len(self._area_history) >= 30 else self._area_history
                if len(recent_areas) >= 10:
                    # Check if most recent areas are very high (close to max)
                    # Higher threshold: 307000 = 99.9% of frame (person fills almost entire frame)
                    close_area_count = sum(1 for a in recent_areas if a > 307000)  # 99.9% of frame
                    close_area_ratio = close_area_count / len(recent_areas)
                    
                    # Also check average area for stability
                    avg_recent_area = sum(recent_areas) / len(recent_areas)
                    
                    # If 80% of recent measurements are > 307000 AND average is > 307000 AND followed for at least 30 seconds
                    # This means person consistently fills almost entire frame (99.9%+) - robot is very close
                    # Require longer follow duration to ensure robot actually moved closer
                    if close_area_ratio > 0.8 and avg_recent_area > 307000 and follow_duration > 30.0:
                        # Person fills almost entire frame consistently - very close
                        self.state = "WAIT"
                        self.wait_start = time.time()
                        self.follow_start = None
                        self._area_history = []  # Reset history
                        self.get_logger().info(f'Person {self.current_id} very close (area={area:.0f}, avg={avg_recent_area:.0f}, close_ratio={close_area_ratio:.2f}, followed for {follow_duration:.1f}s) -> Switching to WAIT')
                    else:
                        # Keep following - log progress occasionally
                        if not hasattr(self, '_follow_progress_count'):
                            self._follow_progress_count = 0
                        self._follow_progress_count += 1
                        if self._follow_progress_count % 100 == 0:  # Log every 10 seconds
                            self.get_logger().info(f'FOLLOW: Person {self.current_id}, area={area:.0f}, avg={avg_recent_area:.0f}, close_ratio={close_area_ratio:.2f}, duration={follow_duration:.1f}s - continuing to get closer')
                else:
                    # Not enough history yet - keep following
                    if not hasattr(self, '_follow_early_count'):
                        self._follow_early_count = 0
                    self._follow_early_count += 1
                    if self._follow_early_count % 50 == 0:  # Log every 5 seconds
                        self.get_logger().info(f'FOLLOW: Person {self.current_id}, area={area:.0f}, duration={follow_duration:.1f}s - building area history...')
            else:
                # Person lost - return to SEARCH
                lost_id = self.current_id
                self.current_id = None
                self.state = "SEARCH"
                self.follow_start = None
                self.get_logger().info(f'Person {lost_id} lost -> Switching to SEARCH')
        
        elif self.state == "WAIT":
            # Stop and wait for 5 seconds (as requested by user)
            cmd = String()
            cmd.data = "STOP"
            self.pub.publish(cmd)
            
            # Check if 5 seconds have passed
            wait_duration = time.time() - self.wait_start if self.wait_start else 0
            if self.wait_start and wait_duration > 5.0:
                # Mark person as completed
                if self.current_id is not None:
                    self.completed_ids.add(self.current_id)
                    self.get_logger().info(f'Person {self.current_id} completed. Total completed: {len(self.completed_ids)}')
                
                # Return to SEARCH
                self.current_id = None
                self.state = "SEARCH"
                self.wait_start = None
                self.get_logger().info('Returning to SEARCH mode (will rotate to find next person)')
            else:
                # Still waiting - log occasionally
                if not hasattr(self, '_wait_log_count'):
                    self._wait_log_count = 0
                self._wait_log_count += 1
                if self._wait_log_count % 50 == 0:  # Log every 5 seconds
                    self.get_logger().info(f'WAIT: Person {self.current_id}, waiting {wait_duration:.1f}s / 5.0s')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    
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

