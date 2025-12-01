#!/usr/bin/env python3
"""
YOLOv8 + DeepSORT Tracker Node for ROS2

This node:
- Subscribes to camera images from TurtleBot3 Waffle
- Runs YOLOv8 object detection
- Uses DeepSORT for multi-object tracking
- Publishes target object position information

Dependencies (install via pip):
    pip install ultralytics
    pip install deep-sort-realtime
    pip install opencv-python
    pip install numpy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_following_interfaces.msg import ObjectPosition
from cv_bridge import CvBridge
import cv2
import numpy as np

# YOLOv8 and DeepSORT imports
# Note: Install these packages:
#   pip install ultralytics
#   pip install deep-sort-realtime
try:
    from ultralytics import YOLO
    from deep_sort_realtime.deepsort_tracker import DeepSort
except ImportError as e:
    print(f"Error importing required packages: {e}")
    print("Please install: pip install ultralytics deep-sort-realtime")
    raise


class YOLOv8TrackerNode(Node):
    """
    ROS2 node that performs object detection and tracking using YOLOv8 and DeepSORT.
    """

    def __init__(self):
        super().__init__('yolov8_tracker_node')
        
        # Declare ROS2 parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('target_class_name', 'person')  # Per spec: ONLY "person" OR "box"
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('model_path', 'yolov8n.pt')  # Can use yolov8n.pt, yolov8s.pt, etc.
        self.declare_parameter('max_age', 30)  # DeepSORT: max frames to keep lost tracks
        self.declare_parameter('n_init', 3)   # DeepSORT: min detections before confirming track
        
        # Get parameter values
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        target_class_param = self.get_parameter('target_class_name').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        max_age = self.get_parameter('max_age').get_parameter_value().integer_value
        n_init = self.get_parameter('n_init').get_parameter_value().integer_value
        
        # Per spec: Only detect ONE target class - "person" OR "box"
        # But YOLOv8 detects: person, chair, dining table, bottle, backpack, suitcase, etc.
        # "box" might not be detected - YOLOv8 uses "suitcase" or "backpack" for boxes
        # Normalize to lowercase for matching
        target_class = target_class_param.strip().lower()
        
        # Map "box" to classes YOLOv8 can detect
        if target_class == 'box':
            # YOLOv8 doesn't have "box" class, but detects boxes as "suitcase" or "backpack"
            # For now, let's use "chair" or "dining table" which are definitely detectable
            self.get_logger().warn('YOLOv8 does not detect "box" class. Using "chair" instead (easily detectable).')
            target_class = 'chair'
        elif target_class not in ['person', 'box', 'chair', 'dining table', 'bottle', 'backpack', 'suitcase']:
            self.get_logger().warn(f'Target class "{target_class}" might not be detectable. Using "chair" instead.')
            target_class = 'chair'
        
        self.target_class = target_class  # Single target class only
        # For "box", also check suitcase and backpack
        if target_class == 'chair':
            self.target_classes = ['chair']  # YOLOv8 detects "chair"
        elif target_class == 'person':
            self.target_classes = ['person']  # YOLOv8 detects "person"
        else:
            self.target_classes = [target_class]  # Keep as list for compatibility with existing code
        
        self.get_logger().info(f'Initializing YOLOv8 Tracker Node...')
        self.get_logger().info(f'Camera topic: {camera_topic}')
        self.get_logger().info(f'Target classes: {self.target_classes}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        
        # Initialize CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Load YOLOv8 model
        try:
            self.get_logger().info(f'Loading YOLOv8 model from: {model_path}')
            self.yolo_model = YOLO(model_path)
            self.get_logger().info('YOLOv8 model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8 model: {e}')
            raise
        
        # Initialize DeepSORT tracker
        try:
            self.tracker = DeepSort(
                max_age=max_age,
                n_init=n_init,
                nms_max_overlap=1.0
            )
            self.get_logger().info('DeepSORT tracker initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize DeepSORT tracker: {e}')
            raise
        
        # Track the main target (most persistent or highest confidence)
        self.main_target_track_id = None
        self.target_track_history = {}  # track_id -> detection_count
        self.track_id_to_class = {}  # track_id -> class_name (for priority selection)
        
        # Create publisher for object position
        self.position_publisher = self.create_publisher(
            ObjectPosition,
            '/object_position',
            10
        )
        
        # Create subscriber for camera images
        # Try multiple possible topic names (Webots might use nested topics)
        self.image_subscriber = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Also try subscribing to the nested topic if main topic doesn't work
        # Webots sometimes publishes to /camera/image_raw/image_color
        self.image_subscriber_alt = None
        if camera_topic == '/camera/image_raw':
            alt_topic = '/camera/image_raw/image_color'
            self.image_subscriber_alt = self.create_subscription(
                Image,
                alt_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f'Also subscribed to alternative topic: {alt_topic}')
        
        # Add timer to check if we're receiving images
        self.image_check_timer = self.create_timer(5.0, self.check_image_reception)
        self.last_image_time = None
        self._image_count = 0
        
        self.get_logger().info('YOLOv8 Tracker Node initialized and ready!')
        self.get_logger().info(f'Subscribed to camera topic: {camera_topic}')
        self.get_logger().info(f'Publishing to: /object_position')
        self.get_logger().info(f'Waiting for images from: {camera_topic}')
        self.get_logger().info(f'Subscribed to camera topic: {camera_topic}')
        self.get_logger().info(f'Publishing to: /object_position')
    
    def image_callback(self, msg):
        """
        Callback function for processing incoming camera images.
        
        Args:
            msg: sensor_msgs/Image message
        """
        try:
            # Log that we received an image (first few times only to avoid spam)
            if not hasattr(self, '_image_count'):
                self._image_count = 0
            self._image_count += 1
            # Update last image time
            self.last_image_time = self.get_clock().now()
            
            if self._image_count <= 10 or self._image_count % 30 == 0:
                self.get_logger().info(f'Received image #{self._image_count} (size: {msg.width}x{msg.height})')
            
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_height, image_width = cv_image.shape[:2]
            
            # Run YOLOv8 detection
            results = self.yolo_model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            # Debug: Log all detected classes (first few frames)
            if not hasattr(self, '_debug_log_count'):
                self._debug_log_count = 0
            self._debug_log_count += 1
            if self._debug_log_count <= 5 or self._debug_log_count % 50 == 0:
                all_classes = []
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.yolo_model.names[cls_id]
                        conf = float(box.conf[0])
                        all_classes.append(f"{cls_name}({conf:.2f})")
                if all_classes:
                    self.get_logger().info(f'YOLOv8 detected: {", ".join(all_classes)}')
                else:
                    self.get_logger().info(f'YOLOv8: No detections (target={self.target_classes})')
            
            # Extract detections for target classes
            detections = []
            detection_classes = []  # Store class info for each detection
            class_names = self.yolo_model.names
            
            # Find class IDs for all target classes
            target_class_ids = {}
            for class_id, class_name in class_names.items():
                class_name_lower = class_name.lower()
                if class_name_lower in self.target_classes:
                    target_class_ids[class_id] = class_name_lower
            
            if len(target_class_ids) == 0:
                self.get_logger().warn(
                    f'No target classes found in YOLOv8 classes. '
                    f'Requested: {self.target_classes}, '
                    f'Available: {list(class_names.values())}'
                )
                self.publish_no_detection(msg.header)
                return
            
            # Extract bounding boxes and confidences for target classes
            # Store detection info with bounding box for class matching
            detection_info = []  # List of (bbox, class_name, confidence)
            target_detections_found = 0
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    class_id = int(box.cls)
                    # Check if this detection is one of the target classes
                    if class_id in target_class_ids:
                        target_detections_found += 1
                        # Get bounding box coordinates (x1, y1, x2, y2)
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf[0].cpu().numpy())
                        class_name = target_class_ids[class_id]
                        
                        # Convert to format expected by DeepSORT: [x1, y1, x2, y2, confidence]
                        detections.append([x1, y1, x2, y2, confidence])
                        detection_info.append(((x1, y1, x2, y2), class_name, confidence))
            
            # Log if target class detections found
            if target_detections_found > 0 and (self._debug_log_count <= 5 or self._debug_log_count % 30 == 0):
                self.get_logger().info(f'Found {target_detections_found} target detection(s) of class: {self.target_classes}')
            
            # Update DeepSORT tracker with detections
            if len(detections) > 0:
                tracks = self.tracker.update_tracks(detections, frame=cv_image)
            else:
                tracks = self.tracker.update_tracks([], frame=cv_image)
            
            # Map tracks to classes by finding closest detection
            for track in tracks:
                if track.is_confirmed():
                    track_id = track.track_id
                    ltrb = track.to_ltrb()
                    tx1, ty1, tx2, ty2 = ltrb
                    track_center = ((tx1 + tx2) / 2, (ty1 + ty2) / 2)
                    
                    # Find closest detection to this track
                    best_match = None
                    min_distance = float('inf')
                    for (dx1, dy1, dx2, dy2), class_name, conf in detection_info:
                        det_center = ((dx1 + dx2) / 2, (dy1 + dy2) / 2)
                        # Calculate distance between centers
                        distance = ((track_center[0] - det_center[0])**2 + 
                                   (track_center[1] - det_center[1])**2)**0.5
                        # Also check overlap
                        overlap = max(0, min(tx2, dx2) - max(tx1, dx1)) * max(0, min(ty2, dy2) - max(ty1, dy1))
                        if overlap > 0 or distance < min_distance:
                            if distance < min_distance:
                                min_distance = distance
                                best_match = class_name
                    
                    if best_match:
                        self.track_id_to_class[track_id] = best_match
            
            # Select main target and publish
            if len(tracks) > 0:
                # Select the best target (most persistent track or highest priority class)
                best_track = self.select_main_target(tracks)
                
                if best_track is not None:
                    # Get bounding box from track
                    if best_track.is_confirmed():
                        ltrb = best_track.to_ltrb()
                        x1, y1, x2, y2 = ltrb
                        track_id = best_track.track_id
                        confidence = best_track.get_det_conf() if hasattr(best_track, 'get_det_conf') else 0.5
                        
                        # Update main target tracking
                        self.main_target_track_id = track_id
                        if track_id not in self.target_track_history:
                            self.target_track_history[track_id] = 0
                        self.target_track_history[track_id] += 1
                        
                        # Normalize coordinates to [0, 1]
                        x_center = (x1 + x2) / 2.0
                        y_center = (y1 + y2) / 2.0
                        width = x2 - x1
                        height = y2 - y1
                        
                        x_norm = x_center / image_width
                        y_norm = y_center / image_height
                        width_norm = width / image_width
                        height_norm = height / image_height
                        
                        # Log detection (occasionally to avoid spam)
                        if not hasattr(self, '_detect_log_count'):
                            self._detect_log_count = 0
                        self._detect_log_count += 1
                        if self._detect_log_count <= 3 or self._detect_log_count % 30 == 0:
                            best_track_class = self.track_id_to_class.get(track_id, 'unknown')
                            self.get_logger().info(
                                f'DETECTED: class={best_track_class}, '
                                f'track_id={track_id}, '
                                f'position=({x_norm:.2f}, {y_norm:.2f}), '
                                f'confidence={confidence:.2f}'
                            )
                        
                        # Publish detection
                        self.publish_detection(
                            msg.header,
                            x_norm, y_norm, width_norm, height_norm,
                            confidence, track_id
                        )
                    else:
                        # Track not confirmed yet
                        self.publish_no_detection(msg.header)
                else:
                    self.publish_no_detection(msg.header)
            else:
                # No tracks found
                self.main_target_track_id = None
                self.publish_no_detection(msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            self.publish_no_detection(msg.header)
    
    def select_main_target(self, tracks):
        """
        Select the main target to follow from multiple tracks.
        
        Per spec: Only one target class (person OR box), so we select:
        1. Prefer the track that matches our main_target_track_id (persistence)
        2. Otherwise, select highest confidence track of the target class
        
        Args:
            tracks: List of DeepSORT tracks
            
        Returns:
            Best track object or None
        """
        confirmed_tracks = [t for t in tracks if t.is_confirmed()]
        
        if len(confirmed_tracks) == 0:
            return None
        
        # Filter tracks to only our target class
        target_class_tracks = []
        for track in confirmed_tracks:
            track_id = track.track_id
            track_class = self.track_id_to_class.get(track_id, None)
            # Only consider tracks of our target class
            if track_class == self.target_class:
                target_class_tracks.append(track)
        
        if len(target_class_tracks) == 0:
            return None
        
        # If we have a main target, try to keep following it
        if self.main_target_track_id is not None:
            for track in target_class_tracks:
                if track.track_id == self.main_target_track_id:
                    return track
        
        # Otherwise, select highest confidence track
        best_track = None
        best_confidence = 0.0
        
        for track in target_class_tracks:
            track_id = track.track_id
            # Try to get confidence from track
            if hasattr(track, 'get_det_conf'):
                conf = track.get_det_conf()
            else:
                # Fallback: use track history
                conf = self.target_track_history.get(track_id, 0) / 10.0
            
            if conf > best_confidence:
                best_confidence = conf
                best_track = track
        
        return best_track if best_track is not None else target_class_tracks[0]
    
    def publish_detection(self, header, x_norm, y_norm, width_norm, height_norm, confidence, track_id):
        """
        Publish a detection message with target object information.
        
        Args:
            header: std_msgs/Header from the image message
            x_norm: Normalized x-center [0, 1]
            y_norm: Normalized y-center [0, 1]
            width_norm: Normalized width [0, 1]
            height_norm: Normalized height [0, 1]
            confidence: Detection confidence [0, 1]
            track_id: DeepSORT track ID
        """
        msg = ObjectPosition()
        msg.header = header
        msg.found = True  # Per spec: use 'found' field
        msg.x = float(x_norm)  # Per spec: use 'x' field (normalized)
        msg.y = float(y_norm)  # Per spec: use 'y' field (normalized)
        # Distance estimate: larger bounding box width = closer object
        # Estimate distance inversely proportional to width
        estimated_distance = max(0.1, 1.0 / max(width_norm, 0.1))  # Inverse relationship
        msg.distance = float(estimated_distance)
        
        self.position_publisher.publish(msg)
        self.get_logger().debug(
            f'Published detection: track_id={track_id}, '
            f'center=({x_norm:.2f}, {y_norm:.2f}), conf={confidence:.2f}'
        )
    
    def publish_no_detection(self, header):
        """
        Publish a message indicating no target detected.
        
        Args:
            header: std_msgs/Header from the image message
        """
        # Log occasionally (not every frame to avoid spam)
        if not hasattr(self, '_no_detect_log_count'):
            self._no_detect_log_count = 0
        self._no_detect_log_count += 1
        if self._no_detect_log_count <= 3 or self._no_detect_log_count % 100 == 0:
            self.get_logger().info(f'No target detection (target={self.target_classes}) - count: {self._no_detect_log_count}')
        
        msg = ObjectPosition()
        msg.header = header
        msg.found = False  # Per spec: use 'found' field
        msg.x = 0.0  # Per spec: use 'x' field
        msg.y = 0.0  # Per spec: use 'y' field
        msg.distance = 0.0
        
        self.position_publisher.publish(msg)
    
    def check_image_reception(self):
        """Check if we're receiving images from the camera"""
        if not hasattr(self, 'last_image_time') or self.last_image_time is None:
            self.get_logger().warn('No images received yet! Check camera topic and camera enablement.')
        else:
            time_since_last = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
            if time_since_last > 2.0:
                self.get_logger().warn(f'No images received for {time_since_last:.1f}s! Camera might have stopped.')


def main(args=None):
    """
    Main entry point for the YOLOv8 Tracker Node.
    """
    rclpy.init(args=args)
    
    try:
        node = YOLOv8TrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error running node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

