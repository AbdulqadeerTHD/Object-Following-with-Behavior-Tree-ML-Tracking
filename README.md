# Object Following Robot with Behavior Tree & ML Tracking

An autonomous robot system that follows persons using YOLOv8 object detection, SORT tracking, and a behavior tree state machine. The robot can search for persons, follow them while avoiding obstacles, and re-identify lost targets.

## System Architecture

### ROS2 Nodes

The system consists of three main ROS2 nodes:

1. **`vision_yolo_node`** (`object_following_vision` package)
   - Performs person detection using YOLOv8
   - Tracks persons across frames using SORT algorithm
   - Publishes detected persons with unique IDs

2. **`behavior_manager_node`** (`object_following_bt` package)
   - Implements state machine: SEARCH → FOLLOW → WAIT → SEARCH
   - Manages person tracking and re-identification
   - Publishes behavior commands

3. **`controller_node`** (`object_following_control` package)
   - Converts behavior commands to motor commands (`cmd_vel`)
   - Implements obstacle avoidance using LiDAR
   - Handles velocity clamping and safety limits

### ROS2 Topics

#### Published Topics

- **`/detected_persons`** (`std_msgs/String`)
  - Publisher: `vision_yolo_node`
  - Format: JSON array of person detections
  - Fields: `id` (int), `x_center` (float), `area` (float)
  - Frequency: ~10-30 Hz (depends on camera frame rate)

- **`/behavior_cmd`** (`std_msgs/String`)
  - Publisher: `behavior_manager_node`
  - Format: String command or JSON for FOLLOW
  - Commands: `"ROTATE"`, `"STOP"`, or `{"cmd": "FOLLOW", "x": float, "area": float}`
  - Frequency: 10 Hz

- **`/cmd_vel`** (`geometry_msgs/Twist`)
  - Publisher: `controller_node`
  - Format: Standard ROS2 velocity command
  - Fields: `linear.x` (m/s), `angular.z` (rad/s)
  - Frequency: 10 Hz

#### Subscribed Topics

- **`/camera/image_raw`** (`sensor_msgs/Image`)
  - Subscriber: `vision_yolo_node`
  - Source: Robot camera (Webots simulation)
  - Format: RGB image (640x480)

- **`/scan`** (`sensor_msgs/LaserScan`)
  - Subscriber: `controller_node`
  - Source: LiDAR sensor
  - Used for: Obstacle detection and avoidance

## Behavior Logic

### SEARCH Mode

**Purpose**: Rotate in place to find a person

**Behavior**:
- Robot rotates continuously at 0.8 rad/s
- No forward movement (`linear.x = 0.0`)
- Monitors `/detected_persons` for new persons
- Switches to FOLLOW mode when a new person (not in `completed_ids`) is detected

**State Transition**:
- SEARCH → FOLLOW: When person detected with valid ID not in completed list

### FOLLOW Mode

**Purpose**: Follow the detected person until very close

**Behavior**:
- Moves forward at maximum speed (0.22 m/s) when person is far
- Adjusts angular velocity based on person's `x_center` position
- Slows down slightly when person fills >99.8% of frame
- Checks for obstacles using LiDAR and bypasses them
- Tracks person using SORT ID for re-identification

**Obstacle Avoidance**:
- When obstacle detected (< 0.40m in front):
  - Checks left/right sectors to choose bypass direction
  - Turns at 1.5 rad/s while moving forward at 0.20 m/s
  - Continues bypassing until obstacle cleared (> 0.60m)

**Re-identification**:
- If person temporarily lost, waits 30 cycles (~3 seconds) before switching to SEARCH
- If same person ID detected again, continues following
- Maintains `completed_ids` set to avoid re-following completed persons

**State Transition**:
- FOLLOW → WAIT: When person area > 306000 (99.6% of frame) consistently for 15+ seconds
- FOLLOW → SEARCH: When person lost for >30 cycles

### WAIT Mode

**Purpose**: Stop and wait when person is reached

**Behavior**:
- Stops completely (`linear.x = 0.0`, `angular.z = 0.0`)
- Waits for 5 seconds
- Marks person ID as completed
- Returns to SEARCH mode to find next person

**State Transition**:
- WAIT → SEARCH: After 5 seconds of waiting

## Technical Details

### Person Detection

- **Model**: Custom trained YOLOv8 model (`person_obstacle_detector`)
- **Confidence Threshold**: 0.05 (very low to catch all detections)
- **Classes**: Person (class 0)
- **Tracking**: SORT (Simple Online and Realtime Tracking)
- **Frame Size**: 640x480 pixels
- **Max Area**: 307200 pixels (full frame)

### Obstacle Avoidance

- **Sensor**: LiDAR (360° scan)
- **Detection Sector**: Front 30-70% of scan (~144°)
- **Obstacle Threshold**: 0.40m (40cm safety distance)
- **Clear Threshold**: 0.60m (60cm to consider obstacle cleared)
- **Bypass Strategy**: Turn left/right based on clearer path, continue forward

### Velocity Control

- **Max Linear Speed**: 0.22 m/s (TurtleBot3 Burger limit)
- **Max Angular Speed**: 2.0 rad/s
- **Search Rotation**: 0.8 rad/s
- **Obstacle Bypass**: 1.5 rad/s angular, 0.20 m/s linear
- **Close Approach**: 0.18 m/s when person fills >99.8% of frame

### State Machine Parameters

- **WAIT Duration**: 5.0 seconds
- **Close Area Threshold**: 306000 pixels (99.6% of frame)
- **Close Area Ratio**: 90% of recent measurements must exceed threshold
- **Follow Duration Requirement**: 15.0 seconds minimum
- **Person Lost Tolerance**: 30 cycles (~3 seconds)

## File Structure

```
ros2_ws/
├── src/
│   ├── object_following_vision/          # Vision and detection package
│   │   └── object_following_vision/
│   │       └── vision_yolo_node.py       # YOLOv8 + SORT tracking
│   ├── object_following_bt/              # Behavior tree package
│   │   └── object_following_bt/
│   │       └── behavior_manager_node.py # State machine logic
│   ├── object_following_control/         # Control package
│   │   └── object_following_control/
│   │       └── controller_node.py        # Motor control + obstacle avoidance
│   └── object_following_integration/      # Integration package
│       └── launch/
│           └── launch_3node_architecture.launch.py
├── runs/
│   └── detect/
│       └── person_obstacle_detector/     # Trained YOLOv8 model
│           └── weights/
│               └── best.pt
└── training_data/                         # Training dataset
    ├── person/                           # Person images
    └── obstacles/                        # Obstacle images
```

## Dependencies

- ROS2 Humble
- Python 3.10+
- ultralytics (YOLOv8)
- sort-track (SORT algorithm)
- cv_bridge
- numpy
- Webots (for simulation)

## Model Training

The system uses a custom-trained YOLOv8 model located at:
```
ros2_ws/runs/detect/person_obstacle_detector/weights/best.pt
```

Training data is stored in `ros2_ws/training_data/` with separate folders for:
- `person/`: Person images for training
- `obstacles/`: Obstacle images for training

## License

MIT License

Copyright (c) 2026 AbdulqadeerTHD

