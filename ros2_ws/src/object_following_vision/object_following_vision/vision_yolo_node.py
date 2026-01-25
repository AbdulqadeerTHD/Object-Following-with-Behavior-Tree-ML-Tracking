#!/usr/bin/env python3
"""
Vision Node: YOLOv8 + SORT Tracker
Detects persons and tracks IDs, publishes to /detected_persons
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np

# YOLOv8 and SORT imports
try:
    from ultralytics import YOLO
    from sort import SortTracker
except ImportError as e:
    print(f"Error importing required packages: {e}")
    print("Please install: pip install ultralytics sort-track")
    raise


class VisionYoloNode(Node):
    """ROS2 node that performs person detection and tracking using YOLOv8 + SORT"""
    
    def __init__(self):
        super().__init__('vision_yolo_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLOv8 model - use trained custom model
        import os
        workspace_root = os.path.expanduser("~/Documents/Path_Following_Robot/ros2_ws")
        model_path = os.path.join(workspace_root, "runs/detect/person_obstacle_detector/weights/best.pt")
        
        try:
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                self.get_logger().info(f'Custom trained YOLOv8 model loaded: {model_path}')
            else:
                raise FileNotFoundError(f"Model not found: {model_path}")
        except Exception as e:
            # Fallback to default model if trained model not found
            self.get_logger().warn(f'Could not load trained model, using default: {e}')
            self.model = YOLO("yolov8n.pt")
            self.get_logger().info('Using default YOLOv8 model (yolov8n.pt)')
        
        # Initialize SORT tracker
        self.tracker = SortTracker()
        self.get_logger().info('SORT tracker initialized')
        
        # Subscribe to camera images - use sensor_data QoS (same as data_collector)
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data  # BEST_EFFORT QoS for camera images
        )
        self.get_logger().info('Subscribed to /camera/image_raw with sensor_data QoS')
        
        # Also try alternative topic in case Webots uses nested topic
        self.sub_alt = self.create_subscription(
            Image,
            '/camera/image_raw/image_color',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Also subscribed to /camera/image_raw/image_color')
        
        # Publish detected persons
        self.pub = self.create_publisher(
            String,
            '/detected_persons',
            10
        )
        self.get_logger().info('Publishing to /detected_persons')
        
        # Timer to check if receiving images
        self.check_timer = self.create_timer(5.0, self.check_image_reception)
        self.last_image_time = None
        
        self.get_logger().info('Vision YOLO Node initialized')
    
    def check_image_reception(self):
        """Check if we're receiving camera images"""
        if self.last_image_time is None:
            self.get_logger().warn('No camera images received yet!')
            self.get_logger().warn('Check: ros2 topic echo /camera/image_raw --once')
            self.get_logger().warn('Check: ros2 topic hz /camera/image_raw')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Update last image reception time
            self.last_image_time = self.get_clock().now()
            
            # Track frame count
            if not hasattr(self, '_frame_count'):
                self._frame_count = 0
                self.get_logger().info('First camera image received!')
            self._frame_count += 1
            
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame_height, frame_width = frame.shape[:2]
            
            # Log first few frames
            if self._frame_count <= 3:
                self.get_logger().info(f'Processing frame {self._frame_count} (size: {frame_width}x{frame_height})')
            
            # Run YOLOv8 detection (class 0 = person from trained model)
            # Very low confidence threshold to detect partially occluded persons behind obstacles
            # Try without class filter first to see all detections, then filter
            results = self.model(frame, conf=0.05, verbose=False)
            
            # Filter for person class (class 0) after detection
            person_detections = []
            for r in results:
                for box in r.boxes:
                    class_id = int(box.cls[0].cpu().numpy())
                    if class_id == 0:  # Only person class
                        person_detections.append(box)
            
            # Log all detections for debugging
            if self._frame_count <= 5:
                all_classes = []
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0].cpu().numpy())
                        all_classes.append(cls_id)
                if len(all_classes) > 0:
                    self.get_logger().info(f'Frame {self._frame_count}: All detections - classes: {all_classes}, person detections: {len(person_detections)}')
            
            # Use filtered person detections
            detections = []
            for box in person_detections:
                # Get bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                score = float(box.conf[0].cpu().numpy())
                class_id = 0  # Person class
                
                detections.append([x1, y1, x2, y2, score, class_id])
            
            # Update SORT tracker (requires detections array and second argument)
            # SortTracker returns: [x1, y1, x2, y2, track_id, class_id, score]
            if len(detections) > 0:
                tracked = self.tracker.update(np.array(detections), None)
            else:
                tracked = self.tracker.update(np.empty((0, 6)), None)
            
            # Format tracked persons
            persons = []
            for t in tracked:
                x1, y1, x2, y2, track_id, class_id, score = t
                cx = (x1 + x2) / 2.0  # Center x
                area = (x2 - x1) * (y2 - y1)  # Bounding box area
                
                persons.append({
                    "id": int(track_id),
                    "x_center": float(cx),
                    "area": float(area)
                })
            
            # Debug logging
            if len(detections) > 0:
                # Always log detections
                self.get_logger().info(f'Frame {self._frame_count}: Detected {len(detections)} person(s), tracked {len(persons)} person(s)')
                for p in persons:
                    self.get_logger().info(f'  Person ID {p["id"]}: x_center={p["x_center"]:.1f}, area={p["area"]:.0f}')
            elif self._frame_count <= 10 or self._frame_count % 50 == 0:
                self.get_logger().warn(f'Frame {self._frame_count}: No person detections - check if person is visible in camera')
            
            # Publish as JSON string
            msg_out = String()
            msg_out.data = json.dumps(persons)
            self.pub.publish(msg_out)
            
            if len(persons) > 0 and self._frame_count <= 10:
                self.get_logger().info(f'Published {len(persons)} person(s) to /detected_persons')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}', exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    node = VisionYoloNode()
    
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

