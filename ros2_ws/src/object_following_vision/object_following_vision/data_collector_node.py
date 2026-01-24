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
        self.declare_parameter('save_interval', 5)  # Save every 5 frames
        self.declare_parameter('output_dir', 'dataset/images')
        self.declare_parameter('max_images', 600)  # Maximum images to collect
        
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
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Data Collector Node initialized')
        self.get_logger().info(f'Camera topic: {camera_topic}')
        self.get_logger().info(f'Save interval: {self.save_interval} frames')
        self.get_logger().info(f'Output directory: {self.output_dir.absolute()}')
        self.get_logger().info(f'Max images: {self.max_images}')
        self.get_logger().info('Starting data collection...')
        self.get_logger().info('Move robot slowly with teleop to collect diverse images')
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        if self.saved_count >= self.max_images:
            if self.saved_count == self.max_images:
                self.get_logger().warn(f'Reached maximum image count ({self.max_images}). Stopping collection.')
                self.saved_count += 1  # Prevent repeated warnings
            return
        
        self.frame_count += 1
        
        # Save every N frames
        if self.frame_count % self.save_interval != 0:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check for duplicates (simple comparison)
            if self.last_image is not None:
                diff = cv2.absdiff(cv_image, self.last_image)
                if np.sum(diff) < 1000:  # Very similar images
                    return
            
            self.last_image = cv_image.copy()
            
            # Generate filename
            filename = f'img_{self.saved_count + 1:06d}.jpg'
            filepath = self.output_dir / filename
            
            # Save image
            cv2.imwrite(str(filepath), cv_image)
            self.saved_count += 1
            
            if self.saved_count % 50 == 0:
                self.get_logger().info(f'Saved {self.saved_count} images...')
            else:
                self.get_logger().debug(f'Saved image {self.saved_count}: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Data collection stopped. Total images saved: {node.saved_count}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



