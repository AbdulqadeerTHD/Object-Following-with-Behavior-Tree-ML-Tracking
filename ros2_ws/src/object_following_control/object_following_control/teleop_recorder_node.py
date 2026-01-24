#!/usr/bin/env python3
"""
Teleop Command Recorder/Replayer Node

Records teleop commands from /cmd_vel and can replay them automatically.
This is a fallback technique if autonomous behavior doesn't work.

Usage:
    # Record commands:
    ros2 run object_following_control teleop_recorder --mode record --output teleop_commands.json
    
    # Replay commands:
    ros2 run object_following_control teleop_recorder --mode replay --input teleop_commands.json
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import sys
from pathlib import Path


class TeleopRecorderNode(Node):
    """Node to record or replay teleop commands"""
    
    def __init__(self):
        super().__init__('teleop_recorder_node')
        
        # Parse command line arguments first (before ROS2 parameters)
        mode = 'record'
        input_file = 'teleop_commands.json'
        output_file = 'teleop_commands.json'
        replay_speed = 1.0  # Changed from 3.0 to 1.0 - EXACT REPLAY (same speed as recorded)
        
        # Parse command line arguments
        args = sys.argv[1:] if len(sys.argv) > 1 else []
        i = 0
        while i < len(args):
            if args[i] == '--mode' and i + 1 < len(args):
                mode = args[i + 1]
                i += 2
            elif args[i] == '--input' and i + 1 < len(args):
                input_file = args[i + 1]
                i += 2
            elif args[i] == '--output' and i + 1 < len(args):
                output_file = args[i + 1]
                i += 2
            elif args[i] == '--replay_speed' and i + 1 < len(args):
                try:
                    replay_speed = float(args[i + 1])
                except ValueError:
                    pass
                i += 2
            else:
                i += 1
        
        # Also check ROS2 parameters as fallback
        self.declare_parameter('mode', mode)
        self.declare_parameter('input_file', input_file)
        self.declare_parameter('output_file', output_file)
        self.declare_parameter('replay_speed', 1.0)  # Changed to 1.0 - EXACT REPLAY
        
        # Use parsed values or ROS2 parameters
        mode = self.get_parameter('mode').get_parameter_value().string_value
        input_file = self.get_parameter('input_file').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.replay_speed = self.get_parameter('replay_speed').get_parameter_value().double_value
        
        self.mode = mode
        
        if mode == 'record':
            # Use high-precision time for recording
            self.start_time = time.perf_counter()  # High precision system time
            self.recorded_commands = []
            self.subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.record_callback,
                10
            )
            self.output_file = Path(output_file)
            self.get_logger().info(f'Recording teleop commands to {self.output_file}')
            self.get_logger().info('Press Ctrl+C to stop recording')
            self.get_logger().info('Recording with high-precision timing (millisecond accuracy)')
            
        elif mode == 'replay':
            self.input_file = Path(input_file)
            if not self.input_file.exists():
                self.get_logger().error(f'Input file not found: {self.input_file}')
                return
            
            self.publisher = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            
            # Load recorded commands
            with open(self.input_file, 'r') as f:
                data = json.load(f)
                self.recorded_commands = data.get('commands', [])
            
            if len(self.recorded_commands) == 0:
                self.get_logger().error('No commands found in file!')
                return
            
            # Preserve exact timing - normalize to start from 0 but keep relative intervals
            if self.recorded_commands:
                self.start_time_offset = self.recorded_commands[0]['time']
                # Adjust all times to start from 0, preserving exact intervals
                for cmd in self.recorded_commands:
                    cmd['time'] = cmd['time'] - self.start_time_offset
            
            self.get_logger().info(f'Loaded {len(self.recorded_commands)} commands from {self.input_file}')
            self.get_logger().info(f'Duration: {self.recorded_commands[-1]["time"]:.6f} seconds (high precision)')
            self.get_logger().info(f'Replay speed: {self.replay_speed}x (1.0 = EXACT SPEED, 2.0 = 2x faster, etc.)')
            if self.replay_speed == 1.0:
                self.get_logger().info('⭐ EXACT REPLAY MODE: Robot will move at EXACTLY the same speed as recorded!')
            self.get_logger().info('Using EXACT timing from recorded commands (microsecond precision)')
            
            # Start replay timer - use high frequency for precise timing
            self.replay_index = 0
            self.replay_start_time = time.perf_counter()  # High precision system time
            self.timer = self.create_timer(0.0005, self.replay_callback)  # 2000Hz for microsecond precision
            
        else:
            self.get_logger().error(f'Invalid mode: {mode}. Use "record" or "replay"')
    
    def record_callback(self, msg):
        """Record incoming teleop commands with high-precision timing"""
        # Use high-precision time for exact recording
        current_time = time.perf_counter()
        elapsed = current_time - self.start_time  # Exact time difference in seconds
        
        # Record ALL commands (including zero) to preserve exact timing
        # This ensures we capture the exact moment when commands change
        command = {
            'time': round(elapsed, 9),  # Round to nanosecond precision (9 decimal places)
            'linear_x': round(float(msg.linear.x), 6),  # Round to reasonable precision
            'angular_z': round(float(msg.angular.z), 6)
        }
        self.recorded_commands.append(command)
        
        # Log speed being recorded
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.get_logger().info(f'Recording: linear_x={msg.linear.x:.4f}, angular_z={msg.angular.z:.4f}, time={elapsed:.6f}s')
    
    def replay_callback(self):
        """Replay recorded commands with EXACT timing - no drift, no delay"""
        if self.replay_index >= len(self.recorded_commands):
            self.get_logger().info('✓ Replay complete! Robot performed exact replay of recorded motion.')
            self.timer.cancel()
            return
        
        # Calculate elapsed time with high precision - NO ACCUMULATION
        # Reset timing to avoid drift
        current_elapsed = (time.perf_counter() - self.replay_start_time) * self.replay_speed
        
        # Get current command to check
        if self.replay_index < len(self.recorded_commands):
            cmd = self.recorded_commands[self.replay_index]
            
            # Publish if time has arrived - EXACT match, not just <= 
            if cmd['time'] <= current_elapsed:
                twist = Twist()
                twist.linear.x = cmd['linear_x']
                twist.angular.z = cmd['angular_z']
                self.publisher.publish(twist)
                
                # Debug log for first few commands to verify timing
                if self.replay_index < 5:
                    self.get_logger().info(f'Published cmd {self.replay_index}: linear={cmd["linear_x"]:.4f}, angular={cmd["angular_z"]:.4f}, recorded_time={cmd["time"]:.6f}, elapsed={current_elapsed:.6f}')
                
                self.replay_index += 1
        
        # Log progress periodically
        if self.replay_index % 500 == 0 and self.replay_index > 0:
            progress = (self.replay_index / len(self.recorded_commands)) * 100
            self.get_logger().info(f'Replay progress: {progress:.1f}% ({self.replay_index}/{len(self.recorded_commands)} commands)')
    
    def save_recording(self):
        """Save recorded commands to file"""
        if self.mode == 'record' and len(self.recorded_commands) > 0:
            data = {
                'total_commands': len(self.recorded_commands),
                'duration': self.recorded_commands[-1]['time'] if self.recorded_commands else 0,
                'commands': self.recorded_commands
            }
            with open(self.output_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Saved {len(self.recorded_commands)} commands to {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.mode == 'record':
            node.save_recording()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

