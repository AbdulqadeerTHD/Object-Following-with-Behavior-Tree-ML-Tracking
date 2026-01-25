#!/usr/bin/env python3
"""
Data Collection Node for YOLOv8 Training

Subscribes to /camera/image_raw and saves images automatically.
Use with manual teleop to collect training data.

Usage:
    ros2 run object_following_vision data_collector
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from pathlib import Path

class DataCollectorNode(Node):
    """Node for automatically collecting training images from camera."""
    
    def __init__(self):
        super().__init__('data_collector_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('save_interval', 1)  # Save EVERY frame (maximum capture)
        self.declare_parameter('output_dir', 'dataset/images')
        self.declare_parameter('max_images', 5000)  # Maximum images (enough for 20 min run)
        
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().integer_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        
        # Create output directory
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Frame counter
        self.frame_count = 0
        self.saved_count = 0
        self.last_image = None
        
        # Subscribe to camera - use sensor_data QoS profile (BEST_EFFORT, designed for camera images)
        # This is the standard QoS for sensor data and should work with Webots camera
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile_sensor_data  # Standard QoS for sensor data (BEST_EFFORT, depth=5)
        )
        self.get_logger().info(f'Subscribed to {camera_topic} with sensor_data QoS profile')
        
        # Timer to check if we're receiving images
        self.last_image_time = self.get_clock().now()
        self.check_timer = self.create_timer(5.0, self.check_image_reception)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('DATA COLLECTOR NODE INITIALIZED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Camera topic: {camera_topic}')
        self.get_logger().info(f'Save interval: {self.save_interval} frame(s)')
        self.get_logger().info(f'Output directory: {self.output_dir.absolute()}')
        self.get_logger().info(f'Max images: {self.max_images}')
        self.get_logger().info('Ready to capture images!')
        self.get_logger().info('Move robot with teleop to start collecting images...')
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        # Update last image reception time
        self.last_image_time = self.get_clock().now()
        
        # CRITICAL: Log EVERY callback to verify it's being called
        if self.frame_count == 0:
            self.get_logger().info('FIRST IMAGE RECEIVED! Callback is working!')
        
        if self.saved_count >= self.max_images:
            if self.saved_count == self.max_images:
                self.get_logger().warn(f'Reached maximum image count ({self.max_images}). Stopping collection.')
                self.saved_count += 1  # Prevent repeated warnings
            return
        
        self.frame_count += 1
        
        # Log first frame received
        if self.frame_count == 1:
            self.get_logger().info('Camera images received! Starting collection...')
        
        # Save every N frames
        if self.frame_count % self.save_interval != 0:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check for duplicates (simple comparison)
            # ALWAYS save first image, then check for duplicates
            if self.last_image is not None:
                diff = cv2.absdiff(cv_image, self.last_image)
                # Very low threshold: 50 for save_interval=1, 100 otherwise
                # This captures images even with very small movements
                similarity_threshold = 50 if self.save_interval == 1 else 100
                diff_sum = np.sum(diff)
                if diff_sum < similarity_threshold:  # Very similar images (robot not moving)
                    if self.frame_count % 200 == 0:  # Log occasionally
                        self.get_logger().debug(f'Frame {self.frame_count}: Skipped duplicate (diff={diff_sum:.0f} < {similarity_threshold})')
                    return
            else:
                # First image - always save it
                self.get_logger().info('First camera image received!')
            
            self.last_image = cv_image.copy()
            
            # Generate filename
            filename = f'img_{self.saved_count + 1:06d}.jpg'
            filepath = self.output_dir / filename
            
            # Save image with high quality
            success = cv2.imwrite(str(filepath), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            if not success:
                self.get_logger().error(f'Failed to save image: {filename}')
                return
            
            self.saved_count += 1
            
            # Log progress every 100 images (more frequent updates)
            if self.saved_count % 100 == 0:
                self.get_logger().info(f'Saved {self.saved_count}/{self.max_images} images to {self.output_dir}')
            elif self.saved_count % 10 == 0:
                self.get_logger().info(f'Saved {self.saved_count} images...')
            elif self.saved_count <= 5:
                self.get_logger().info(f'Image {self.saved_count} saved! ({filename})')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}', exc_info=True)
    
    def check_image_reception(self):
        """Check if we're receiving camera images"""
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_image_time).nanoseconds / 1e9
        
        if self.frame_count == 0:
            self.get_logger().warn('No camera images received yet!')
            self.get_logger().warn('   Check if camera topic is publishing: ros2 topic echo /camera/image_raw --once')
        elif time_since_last > 2.0 and self.saved_count == 0:
            self.get_logger().warn(f'Received {self.frame_count} frames but saved 0 images')
            self.get_logger().warn('   Robot might be still or images too similar. Try moving faster!')


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Data collection stopped. Total images saved: {node.saved_count}')
    except Exception as e:
        node.get_logger().error(f'Error in data collector: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()



