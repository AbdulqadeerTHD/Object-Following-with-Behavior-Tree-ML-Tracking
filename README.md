# AI-Based Object Following Robot - ROS2 Project

## Project Overview

This project implements an AI-based object following robot using ROS2, YOLOv8 + DeepSORT, Behavior Trees, and PID control. The robot can detect, track, and follow target objects in a Webots simulation environment. The system supports detecting **all COCO classes** that YOLOv8 can recognize, including chairs, persons, bottles, tables, and more.

## Architecture

The system consists of four main components:

1. **Vision Node** (`object_following_vision`): YOLOv8 object detection + DeepSORT tracking
2. **Behavior Tree Node** (`object_following_bt`): High-level decision making (SEARCH/FOLLOW states)
3. **Control Node** (`object_following_control`): Motion control with obstacle avoidance
4. **Integration Package** (`object_following_integration`): Launch files and Webots world configuration

## System Topics

- `/camera/image_raw` - Camera images from Webots
- `/scan` - LiDAR data from Webots
- `/object_position` - Detected object position (normalized x, y, distance)
- `/behavior_state` - Current robot state (SEARCH or FOLLOW)
- `/cmd_vel` - Velocity commands to robot

## Prerequisites

- ROS2 Humble
- Webots R2023b or R2025a
- Python 3.10+
- Required Python packages:
  ```bash
  pip install ultralytics deep-sort-realtime opencv-python numpy
  ```

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/AbdulqadeerTHD/AI_Bases_Path_Following_Robot_ROS2_CS.git
   cd AI_Bases_Path_Following_Robot_ROS2_CS
   git checkout dev
   ```

2. Build the workspace:
   ```bash
   cd ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running the System

### Launch the complete system with new environment:

```bash
cd ~/Documents/Path_Following_Robot/ros2_ws
source install/setup.bash
ros2 launch object_following_integration launch_object_following_world.launch.py \
    target_class_name:=all \
    confidence_threshold:=0.15
```

### Parameters:

- `camera_topic`: Camera topic name (default: `/camera/image_raw`)
- `target_class_name`: Target object class to detect - `all`, `chair`, `person`, `bottle`, etc. (default: `all`)
  - Use `all` to detect all COCO classes that YOLOv8 can recognize
- `confidence_threshold`: YOLOv8 confidence threshold 0.0-1.0 (default: `0.2`)

### Example: Detect specific class

```bash
ros2 launch object_following_integration launch_object_following_world.launch.py \
    target_class_name:=chair \
    confidence_threshold:=0.3
```

## World Environment

The simulation includes:
- **TurtleBot3 Burger** robot with camera and LiDAR
- **Moving blue target** (automatically moves back and forth)
- **Multiple objects**: boxes, chairs, bottles, table
- **All objects on ground level** with realistic sizes
- **Obstacles** for testing obstacle avoidance

## Behavior Logic

### SEARCH Mode:
- Robot rotates slowly at **0.3 rad/s** when no target is visible
- Switches to FOLLOW mode immediately when target is detected
- Re-enters SEARCH mode if target is lost for **1.5 seconds**

### FOLLOW Mode:
- **TURNING RULES**:
  - If `x < 0.4` → turn left (angular proportional to `(x - 0.5)`)
  - If `x > 0.6` → turn right (angular proportional to `(x - 0.5)`)
- **FORWARD MOTION**:
  - If `0.4 <= x <= 0.6` → move forward at **0.15 m/s**
  - Angular velocity proportional to `(x - 0.5)` for fine centering

### Obstacle Avoidance (Highest Priority):
- If any LiDAR scan distance < **0.30 m** → STOP immediately
- Overrides both FOLLOW and SEARCH modes
- Robot turns away from obstacle

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
├── object_following_control/         # Motion control node
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
    │   ├── launch_object_following_world.launch.py  # New world launch file
    │   └── launch_with_camera_world.launch.py       # Original launch file
    ├── worlds/
    │   ├── object_following_world.wbt               # New environment
    │   ├── turtlebot3_simple_with_camera.wbt        # Original world
    │   ├── moving_target_controller.py              # Target movement controller
    │   └── controllers/
    │       └── moving_target_controller/
    │           └── moving_target_controller.py
    ├── resource/
    │   └── turtlebot_webots_with_camera.urdf
    ├── package.xml
    └── CMakeLists.txt
```

## Features

### ✅ Working Components:

1. **Camera**: Publishing images successfully to `/camera/image_raw`
2. **Vision Node**: 
   - Detects all COCO classes (when `target_class_name:=all`)
   - DeepSORT tracking for object identity
   - Handles numpy.float32 errors correctly
3. **Behavior Tree Node**: 
   - SEARCH/FOLLOW state management
   - 1.5 second delay before switching to SEARCH mode
   - State stability (no rapid switching)
4. **Control Node**: 
   - Exact behavior per specification
   - Obstacle avoidance with 0.30m threshold
   - Smooth motion control
5. **World Environment**: 
   - Correctly scaled objects (half-size for better detection)
   - All objects on ground level
   - Moving target with automatic motion
   - Multiple detectable objects (chairs, bottles, table, boxes)

## Troubleshooting

### Camera not publishing:
1. Check if camera is enabled in Webots GUI
2. Verify URDF contains camera device reference
3. Check topic: `ros2 topic list | grep camera`

### No object detections:
1. Use `target_class_name:=all` to detect all classes
2. Lower confidence threshold: `confidence_threshold:=0.15`
3. Check if objects are visible in camera view
4. Verify YOLOv8 model is loaded (check logs)

### Robot not moving:
1. Check controllers are activated: `ros2 control list_controllers`
2. Verify `/cmd_vel` is publishing: `ros2 topic echo /cmd_vel`
3. Check for controller errors in launch logs

### Webots warnings:
- **Version compatibility warnings**: Safe to ignore (forward compatibility works)
- **Controller directory warning**: Target won't move automatically, but robot works
- **BallJoint warning**: Safe to ignore (URDF export limitation)

## License

MIT

## Maintainers

Team
