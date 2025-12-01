# AI-Based Object Following Robot - ROS2 Project

## Project Overview

This project implements an AI-based object following robot using ROS2, YOLOv8 + DeepSORT, Behavior Trees, and PID control. The robot can detect, track, and follow target objects (chairs, persons, or boxes) in a Webots simulation environment.

## Architecture

The system consists of four main components:

1. **Vision Node** (`object_following_vision`): YOLOv8 object detection + DeepSORT tracking
2. **Behavior Tree Node** (`object_following_bt`): High-level decision making (SEARCH/FOLLOW states)
3. **Control Node** (`object_following_control`): PID-based motion control with obstacle avoidance
4. **Integration Package** (`object_following_integration`): Launch files and Webots world configuration

## System Topics

- `/camera/image_raw` - Camera images from Webots
- `/scan` - LiDAR data from Webots
- `/object_position` - Detected object position (normalized x, y, distance)
- `/behavior_state` - Current robot state (SEARCH or FOLLOW)
- `/cmd_vel` - Velocity commands to robot

## Prerequisites

- ROS2 Humble
- Webots R2023b or later
- Python 3.10+
- Required Python packages:
  ```bash
  pip install ultralytics deep-sort-realtime opencv-python numpy
  ```

## Installation

1. Clone the repository:
   ```bash
   cd ~/Documents/Path_Following_Robot
   ```

2. Build the workspace:
   ```bash
   cd ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running the System

### Launch the complete system:

```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_with_camera_world.launch.py \
    camera_topic:=/camera/image_raw \
    target_class_name:=chair \
    confidence_threshold:=0.3
```

### Parameters:

- `camera_topic`: Camera topic name (default: `/camera/image_raw`)
- `target_class_name`: Target object class to detect - `chair`, `person`, `box`, etc. (default: `chair`)
- `confidence_threshold`: YOLOv8 confidence threshold 0.0-1.0 (default: `0.4`)

## Testing Commands

### Check camera feed:
```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --once
```

### Monitor object detections:
```bash
ros2 topic echo /object_position
```

### Check behavior state:
```bash
ros2 topic echo /behavior_state
```

### Monitor robot commands:
```bash
ros2 topic echo /cmd_vel
```

### Check all active topics:
```bash
ros2 topic list
```

## Current Status

### Working Components:

1. **Camera**: Publishing images successfully to `/camera/image_raw`
2. **Vision Node**: Receiving images and running YOLOv8 detection
3. **Control Node**: Publishing velocity commands continuously
4. **BT Node**: Publishing behavior state (SEARCH/FOLLOW)
5. **Controllers**: Successfully activated (diffdrive_controller, joint_state_broadcaster)
6. **Robot Motion**: Commands being sent (robot rotating in SEARCH mode)

### Current Issues:

1. **Object Detection**: YOLOv8 is not detecting "chair" objects
   - Status: Vision node receives images but reports "No detections"
   - Cause: Simple box objects in world are not recognized as "chair" by YOLOv8
   - Solution: Need actual chair-shaped objects or use different target class

2. **World Objects**: Current world has simple boxes, not actual chairs
   - Objects placed at ~2 meters distance
   - YOLOv8 requires recognizable objects (chairs, persons, tables, etc.)
   - Simple boxes may not be detected reliably

## Project Structure

```
ros2_ws/src/
├── object_following_vision/          # YOLOv8 + DeepSORT vision node
│   ├── object_following_vision/
│   │   └── yolov8_tracker_node.py
│   ├── package.xml
│   └── setup.py
├── object_following_bt/              # Behavior Tree decision node
│   ├── object_following_bt/
│   │   └── bt_node.py
│   ├── package.xml
│   └── setup.py
├── object_following_control/         # PID control node
│   ├── object_following_control/
│   │   └── control_node.py
│   ├── package.xml
│   └── setup.py
├── object_following_interfaces/      # Custom ROS2 messages
│   ├── msg/
│   │   └── ObjectPosition.msg
│   ├── package.xml
│   └── CMakeLists.txt
└── object_following_integration/      # Launch files and world
    ├── launch/
    │   └── launch_with_camera_world.launch.py
    ├── worlds/
    │   └── turtlebot3_simple_with_camera.wbt
    ├── resource/
    │   └── turtlebot_webots_with_camera.urdf
    ├── package.xml
    └── CMakeLists.txt
```

## Control Logic

### FOLLOW Mode:
- If target_x < 0.4 → turn left
- If target_x > 0.6 → turn right
- If 0.4 <= target_x <= 0.6 → go forward
- Adjust speed based on distance (closer = slower)

### SEARCH Mode:
- Robot rotates slowly (0.3 rad/s) until object detected
- Continues until vision node reports `found=true`

### Obstacle Avoidance:
- If LiDAR reading < 0.3m → STOP or slight turn
- Overrides FOLLOW mode when danger detected

## Troubleshooting

### Camera not publishing:
1. Check if camera is enabled in Webots GUI
2. Verify URDF contains camera device reference
3. Check topic: `ros2 topic list | grep camera`

### No object detections:
1. Verify target class is detectable by YOLOv8 (chair, person, dining table, etc.)
2. Check if objects are visible in camera view
3. Lower confidence threshold if needed
4. Try different target class (e.g., `person` instead of `chair`)

### Robot not moving:
1. Check controllers are activated: `ros2 control list_controllers`
2. Verify `/cmd_vel` is publishing: `ros2 topic echo /cmd_vel`
3. Check for controller errors in launch logs

## License

MIT

## Maintainers

Team

